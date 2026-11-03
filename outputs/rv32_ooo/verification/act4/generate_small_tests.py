#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Use ACT4's own chunk splitter without changing upstream source or test cases.

Run from the pinned /act4 checkout with its uv environment. The only generator
setting changed is an in-process file-size limit. Three copied environment
alignment directives are adapted for this non-C platform; instruction tests and
self-checks are never patched. This script does not compile or simulate tests.
"""

from collections import Counter
from dataclasses import asdict, replace
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import shutil

REVISION = "2dc36dd2f3d25383039effa10f1a3930a78a945a"
SUITES = ("I", "M", "Zicsr")
LIMIT = 128
OUTPUT = Path("/project/build/act4_small/tests")


def sha(data):
    return hashlib.sha256(data).hexdigest()


def stable(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":")).encode()


def inventory(directory):
    cases, labels, literals = Counter(), Counter(), Counter()
    files = {}
    for suite in (*SUITES, "Zifencei"):
        for path in sorted((directory / "rv32i" / suite).glob("*.S")):
            text = path.read_text()
            local = 0
            for line in text.splitlines():
                if "_str:" in line and ".string" in line and "test: " in line:
                    label = line.split("_str:", 1)[0].strip()
                    payload = line.split(".string", 1)[1].strip()
                    cases[f"{label}\t{payload}"] += 1
                    labels[label] += 1
                    local += 1
                match = re.match(r"\s*\.(word|dword|byte|short|quad|octa)\s+([^#]+)", line)
                if match:
                    literals[f"{suite}:{match.group(1)}:{match.group(2).strip()}"] += 1
            files[str(path.relative_to(directory))] = {
                "sha256": sha(path.read_bytes()), "testcases": local}
    return {"cases": cases, "labels": labels, "literals": literals, "files": files}


def compare(left, right):
    return {"equal": left == right, "left_count": sum(left.values()),
            "right_count": sum(right.values()), "left_sha256": sha(stable(left)),
            "right_sha256": sha(stable(right)),
            "missing": dict(left - right), "added": dict(right - left)}


def main():
    import testgen.generate.unpriv as generator
    import testgen.io.writer as writer

    upstream = Path.cwd().resolve()
    # The small official build image need not contain the git executable; its
    # mounted checkout still carries the real revision metadata.
    git_dir = upstream / ".git"
    head = (git_dir / "HEAD").read_text().strip()
    if head.startswith("ref: "):
        ref = head[5:]
        if (git_dir / ref).is_file():
            revision = (git_dir / ref).read_text().strip()
        else:
            revision = next(line.split()[0] for line in
                            (git_dir / "packed-refs").read_text().splitlines()
                            if line.endswith(" " + ref))
    else:
        revision = head
    if revision != REVISION:
        raise SystemExit(f"Expected upstream {REVISION}, found {revision}")
    if not (upstream / "testplans/I.csv").is_file():
        raise SystemExit("Run from the upstream ACT4 repository root")
    OUTPUT.mkdir(parents=True, exist_ok=True)
    baseline = OUTPUT.parent / "baseline_default"
    baseline.mkdir(parents=True, exist_ok=True)
    original_sources = inventory(upstream / "tests")
    watched = [upstream / "testplans" / f"{suite}.csv" for suite in SUITES]
    watched += list((upstream / "generators/testgen/src/testgen").rglob("*.py"))
    watched += list((upstream / "tests/env").rglob("*.h"))
    before = {str(p.relative_to(upstream)): sha(p.read_bytes()) for p in watched}
    captured = []
    current_suite = ""
    original_splitter = generator.split_test_chunks
    original_writer = generator.write_test_file
    original_limit = generator.TESTCASES_PER_FILE
    previous_pointer_state = {}
    boundary_fixes = []
    generation_pass = ""

    def split_and_capture(chunks, limit):
        captured.append({"suite": current_suite, "chunks": len(chunks),
                         "testcases": sum(chunk.num_testcases for chunk in chunks),
                         "largest_chunk": max(chunk.num_testcases for chunk in chunks),
                         "sha256": sha(stable([asdict(chunk) for chunk in chunks]))})
        return original_splitter(chunks, limit)

    def write_with_boundary_state(test_config, instr_name, chunks, output_dir,
                                  file_idx=0, extra_defines=None, **kwargs):
        # A chunk's recorded start state can already reflect its opening `mv`.
        # The PRE-move source register is the preceding chunk's end state.
        # Change only the writer's pointer-initialization metadata on a shallow
        # copy; all generated instruction/check/data lists remain untouched.
        key = (test_config.testsuite, instr_name)
        before_state = previous_pointer_state.get(key, (2, 3)) if file_idx else (2, 3)
        first = chunks[0]
        prefix_before = writer._reinit_pointer_registers(first)
        adjusted = replace(first, start_sig_reg=before_state[0],
                           start_data_reg=before_state[1])
        prefix_after = writer._reinit_pointer_registers(adjusted)
        path = output_dir / f"{test_config.name_prefix}-{instr_name}-{file_idx:02d}.S"
        old_hash = sha(path.read_bytes()) if path.is_file() else None
        result = original_writer(test_config, instr_name, [adjusted, *chunks[1:]],
                                 output_dir, file_idx, extra_defines, **kwargs)
        previous_pointer_state[key] = (chunks[-1].end_sig_reg, chunks[-1].end_data_reg)
        if prefix_before != prefix_after:
            boundary_fixes.append({"pass": generation_pass,
                                   "file": str(path.relative_to(output_dir.parent.parent)),
                                   "file_index": file_idx,
                                   "preceding_logical_sig_reg": before_state[0],
                                   "preceding_logical_data_reg": before_state[1],
                                   "recorded_first_sig_reg": first.start_sig_reg,
                                   "recorded_first_data_reg": first.start_data_reg,
                                   "prefix_before": prefix_before, "prefix_after": prefix_after,
                                   "previous_file_sha256": old_hash,
                                   "corrected_file_sha256": sha(result.read_bytes())})
        return result

    generator.split_test_chunks = split_and_capture
    generator.write_test_file = write_with_boundary_state
    records = {}
    try:
        for label, limit, destination in (("default", original_limit, baseline),
                                           ("small", LIMIT, OUTPUT)):
            generator.TESTCASES_PER_FILE = limit
            captured = []
            previous_pointer_state = {}
            generation_pass = label
            for suite in SUITES:
                current_suite = suite
                generator.generate_unpriv_extension_tests(
                    32, False, suite, upstream / "testplans", destination)
            records[label] = captured
    finally:
        generator.TESTCASES_PER_FILE = original_limit
        generator.split_test_chunks = original_splitter
        generator.write_test_file = original_writer

    # Copy the one retained Zifencei program byte-for-byte. Do not regenerate or
    # edit its self-modifying instruction sequences and ignored-field checks.
    for destination in (baseline, OUTPUT):
        shutil.copytree(upstream / "tests/rv32i/Zifencei",
                        destination / "rv32i/Zifencei", dirs_exist_ok=True)
    shutil.copytree(upstream / "tests/env", OUTPUT / "env", dirs_exist_ok=True)
    patches = []
    for name in ("rvtest_setup.h", "rvtest_trap_handler.h", "utils.h"):
        path = OUTPUT / "env" / name
        old = path.read_text()
        pattern = r"(?m)^([ \t]*\.option[ \t]+)rvc\b"
        matches = list(re.finditer(pattern, old))
        if len(matches) != 1:
            raise RuntimeError(f"Expected exactly one authorized alignment site in {name}")
        new = re.sub(pattern, r"\1norvc", old)
        line = old[:matches[0].start()].count("\n") + 1
        patches.append({"file": f"env/{name}", "line": line,
                        "before": old.splitlines()[line - 1],
                        "after": new.splitlines()[line - 1],
                        "before_sha256": sha(old.encode()), "after_sha256": sha(new.encode()),
                        "reason": "32-bit NOP alignment on a platform without C; no test changed"})
        path.write_text(new)

    default = inventory(baseline)
    small = inventory(OUTPUT)
    comparisons = {
        "upstream_vs_default_cases": compare(original_sources["cases"], default["cases"]),
        "upstream_vs_small_cases": compare(original_sources["cases"], small["cases"]),
        "upstream_vs_small_labels": compare(original_sources["labels"], small["labels"]),
        "upstream_vs_default_data_literals": compare(original_sources["literals"],
                                                      default["literals"]),
        "default_vs_small_data_literals": compare(default["literals"], small["literals"]),
    }
    rvc_sites = []
    for path in OUTPUT.rglob("*"):
        if path.is_file() and path.suffix in (".h", ".S"):
            for number, line in enumerate(path.read_text().splitlines(), 1):
                if re.match(r"\s*\.option\s+rvc\b", line):
                    rvc_sites.append(f"{path.relative_to(OUTPUT)}:{number}")
    after = {str(p.relative_to(upstream)): sha(p.read_bytes()) for p in watched}
    checks = {"all_pre_split_chunks_identical": records["default"] == records["small"],
              "upstream_sources_unchanged": before == after,
              "all_upstream_testcases_retained": comparisons["upstream_vs_small_cases"]["equal"],
              "all_upstream_labels_retained": comparisons["upstream_vs_small_labels"]["equal"],
              "no_generated_operand_changes_from_splitting":
                  comparisons["default_vs_small_data_literals"]["equal"],
              "no_rvc_alignment_sites_remain": not rvc_sites}
    manifest = {"status": "generated_inventory_verified" if all(checks.values()) else "failed",
                "created_utc": datetime.now(timezone.utc).isoformat(),
                "upstream_revision": revision, "generator_module": str(Path(generator.__file__)),
                "original_limit": original_limit, "in_process_limit": LIMIT,
                "upstream_files_modified": False, "checks": checks,
                "seed_policy": "Unchanged upstream seed(reproducible_hash(instruction+coverpoint));"
                               " chunks generated before splitting and hashed before writer resets",
                "data_note": "File boundaries add upstream pointer resets and per-file canaries;"
                             " executable/data addresses and signature layout can change."
                             " All pre-split chunk code, operands and metadata are compared.",
                "boundary_pointer_prefix_adaptation": {
                    "rule": "Initialize pre-first-chunk pointers from preceding logical chunk end;"
                            " file zero starts at signature x2 and data x3",
                    "test_instruction_lists_modified": False,
                    "self_checks_modified": False, "changes": boundary_fixes},
                "original_program_count": len(original_sources["files"]),
                "split_program_count": len(small["files"]),
                "original_files": original_sources["files"], "output_files": small["files"],
                "comparisons": comparisons, "pre_split_chunk_records": records["small"],
                "copied_env_alignment_patches": patches, "remaining_rvc_sites": rvc_sites,
                "input_source_sha256": before,
                "script_sha256": sha(Path(__file__).read_bytes()),
                "official_tests_compiled_by_this_script": 0,
                "official_tests_executed_by_this_script": 0}
    manifest_path = OUTPUT.parent / "generation_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({key: manifest[key] for key in
                     ("status", "original_program_count", "split_program_count", "checks")}, indent=2))
    print(f"Manifest: {manifest_path}")
    if not all(checks.values()):
        raise SystemExit("Inventory verification failed; do not claim complete test preservation")


if __name__ == "__main__":
    main()
