#!/usr/bin/env python3
"""Execute every final .elf under a directory with one compiled core harness.

This runner records program-reported results, not architecture certification.
Optional manifest format (paths are relative to --elf-dir, using forward slashes):
  {"upstream_revision": "...", "config": {...},
   "elfs": {"test.elf": "<64 hexadecimal SHA256 characters>", ...}}
Config may instead be a nonempty string identifying the retained build config.
Filename sets and hashes must match exactly before manifest provenance is used.
Each invocation retains a separate run directory, including every failure log.
"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path, PurePosixPath
import re
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def command_run(command, log, timeout):
    """Retain complete merged output, including partial output on host timeout."""
    command = [str(item) for item in command]
    timed_out = False
    try:
        result = subprocess.run(command, cwd=ROOT, text=True, errors="replace",
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                timeout=timeout, check=False)
        output, exit_code = result.stdout, result.returncode
    except subprocess.TimeoutExpired as error:
        output = error.stdout or b""
        if isinstance(output, bytes):
            output = output.decode(errors="replace")
        output += "\nHOST_TIMEOUT: wall-clock limit exceeded\n"
        exit_code, timed_out = None, True
    except OSError as error:
        output, exit_code = f"HOST_ERROR: {error}\n", None
    log.write_text(output)
    return {"command": command, "exit_code": exit_code,
            "host_timeout": timed_out, "log": str(log), "log_sha256": sha256(log),
            "output": output}


def without_output(result):
    return {key: value for key, value in result.items() if key != "output"}


def verify_manifest(path, inventory):
    """Check the actual ELF inventory before trusting supplied build metadata."""
    result = {"status": "provenance_not_verified", "manifest": None, "errors": []}
    if path is None:
        result["reason"] = "No build manifest supplied; ELF origin is not verified."
        return result
    result["manifest"] = str(path)
    try:
        raw = path.read_bytes()
        manifest = json.loads(raw)
        result["manifest_sha256"] = hashlib.sha256(raw).hexdigest()
        if not isinstance(manifest, dict):
            raise ValueError("manifest root must be an object")
        revision = manifest.get("upstream_revision")
        config = manifest.get("config")
        if not isinstance(revision, str) or not revision.strip():
            raise ValueError("upstream_revision must be a nonempty string")
        if (not isinstance(config, (str, dict)) or not config
                or isinstance(config, str) and not config.strip()):
            raise ValueError("config must be a nonempty string or object")
        records = manifest.get("elfs")
        if not isinstance(records, dict):
            raise ValueError("elfs must map relative ELF filenames to SHA256 values")
        for name, digest in records.items():
            relative = PurePosixPath(name)
            if (not name or relative.is_absolute() or ".." in relative.parts
                    or relative.as_posix() != name or "\\" in name):
                raise ValueError(f"invalid relative ELF filename: {name!r}")
            if not isinstance(digest, str) or not re.fullmatch(r"[0-9a-fA-F]{64}", digest):
                raise ValueError(f"invalid SHA256 for {name}")
        actual = {item["name"]: item["elf_sha256"] for item in inventory}
        missing = sorted(set(actual) - set(records))
        extra = sorted(set(records) - set(actual))
        if missing:
            result["errors"].append({"unlisted_elfs": missing})
        if extra:
            result["errors"].append({"manifest_elfs_not_found": extra})
        mismatches = [name for name in sorted(set(actual) & set(records))
                      if actual[name] != records[name].lower()]
        if mismatches:
            result["errors"].append({"elf_hash_mismatches": mismatches})
        if not result["errors"]:
            result.update({"status": "manifest_inventory_verified",
                           "upstream_revision": revision, "config": config,
                           "note": "Inventory integrity verified; upstream authenticity "
                                   "and architectural certification are not established."})
    except (OSError, ValueError, TypeError) as error:
        result["errors"].append(str(error))
    return result


def classify_simulation(result):
    summaries = re.findall(r"ACT4_HARNESS (?:PASS|FAIL|TIMEOUT|CONFIG|PROTOCOL)[^\n]*",
                           result["output"])
    if result["host_timeout"] or any("ACT4_HARNESS TIMEOUT" in s for s in summaries):
        status = "timeout"
    elif (result["exit_code"] == 0 and len(summaries) == 1
          and summaries[0].startswith("ACT4_HARNESS PASS")):
        status = "passed"
    else:
        status = "failed"
    return status, summaries


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf-dir", type=Path, required=True)
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--tohost", "--mailbox", type=lambda value: int(value, 0),
                        default=0x1000FFF0, help="word-aligned mailbox outside RAM")
    parser.add_argument("--max-cycles", type=int, default=1000000)
    parser.add_argument("--data-latency", type=int, default=2)
    parser.add_argument("--predict", type=int, choices=(0, 1), default=1)
    parser.add_argument("--wall-timeout", type=float, default=120)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "build/act4_suite")
    args = parser.parse_args()
    if not (0x10000 <= args.tohost <= 0xFFFFFFFC and args.tohost % 4 == 0):
        parser.error("--tohost/--mailbox must be word-aligned and outside the 64 KiB RAM")
    if (not 0 < args.max_cycles <= 0x7FFFFFFF
            or not 0 <= args.data_latency <= 0x7FFFFFFF
            or not math.isfinite(args.wall_timeout) or args.wall_timeout <= 0):
        parser.error("limits must be positive; data latency must be nonnegative")
    elf_dir = args.elf_dir.resolve()
    if not elf_dir.is_dir():
        parser.error("--elf-dir must be an existing directory")
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    run_dir = output / f"run-{stamp}"
    run_dir.mkdir()
    files = sorted((path for path in elf_dir.rglob("*")
                    if path.is_file() and path.suffix.lower() == ".elf"),
                   key=lambda path: path.relative_to(elf_dir).as_posix())
    cases = []
    for index, path in enumerate(files):
        name = path.relative_to(elf_dir).as_posix()
        safe_stem = re.sub(r"[^A-Za-z0-9_.-]", "_", path.stem)[:80]
        directory = run_dir / f"{index:05d}_{safe_stem}"
        directory.mkdir()
        case = {"name": name, "elf": str(path), "directory": str(directory),
                "status": "failed", "stage": "inventory"}
        try:
            case.update({"elf_sha256": sha256(path), "elf_bytes": path.stat().st_size})
        except OSError as error:
            case.update({"elf_sha256": None, "reason": str(error)})
            log = directory / "inventory_error.log"
            log.write_text(f"Cannot read ELF: {error}\n")
            case.update({"log": str(log), "log_sha256": sha256(log)})
        cases.append(case)
    manifest = args.manifest.resolve() if args.manifest else None
    provenance = verify_manifest(manifest, cases)
    package = ROOT / "rtl/rv32_ooo_pkg.sv"
    sources = [package] + sorted(p for p in (ROOT / "rtl").glob("*.sv") if p != package)
    sources.append(ROOT / "tb/tb_act4.sv")
    tracked = sources + [ROOT / "scripts/act4_elf_to_mem.py", Path(__file__).resolve()]
    report = {"scope": "batch_execution_of_input_elf_programs", "certification_claim": False,
              "started_utc": stamp, "elf_dir": str(elf_dir), "run_directory": str(run_dir),
              "discovery": "recursive, case-insensitive .elf suffix; no file filters",
              "configuration": {"tohost": f"0x{args.tohost:08x}",
                                "max_cycles": args.max_cycles,
                                "data_latency": args.data_latency, "predict": args.predict,
                                "wall_timeout_seconds": args.wall_timeout},
              "provenance": provenance,
              "source_sha256": {str(p.relative_to(ROOT)): sha256(p) for p in tracked},
              "cases": cases, "errors": [], "tool_versions": {}}
    for tool in ("iverilog", "vvp"):
        executable = shutil.which(tool)
        if not executable:
            report["errors"].append(f"Required tool not found: {tool}")
            continue
        version = command_run([executable, "-V"], run_dir / f"{tool}_version.log", 10)
        report["tool_versions"][tool] = without_output(version)
        report["tool_versions"][tool]["text"] = version["output"].strip()
    report["tool_versions"]["python"] = sys.version
    if not files:
        report["errors"].append("No .elf files found; an empty suite cannot pass")
    executable = run_dir / "tb_act4.vvp"
    compile_result = command_run(["iverilog", "-g2012", "-s", "tb_act4", "-o", executable]
                                 + sources, run_dir / "compile.log", 60)
    report["compile"] = without_output(compile_result)
    if compile_result["exit_code"] != 0:
        report["errors"].append("Harness compilation failed or timed out")
    for case in cases:
        directory = Path(case["directory"])
        if case["elf_sha256"] is None:
            continue
        rows = directory / "image.rows.hex"
        load_report = directory / "load.json"
        conversion = command_run([sys.executable, ROOT / "scripts/act4_elf_to_mem.py",
                                  case["elf"], "--words", directory / "image.words.hex",
                                  "--rows", rows, "--report", load_report],
                                 directory / "conversion.log", 30)
        case["conversion"] = without_output(conversion)
        if conversion["exit_code"] != 0:
            case.update({"status": "conversion_failed", "stage": "conversion"})
        else:
            try:
                loaded = json.loads(load_report.read_text())
                if loaded["elf_sha256"] != case["elf_sha256"]:
                    raise ValueError("ELF changed between inventory and conversion")
                case["load_report"] = str(load_report)
                case["rows_sha256"] = sha256(rows)
            except (OSError, ValueError, KeyError) as error:
                case.update({"status": "conversion_failed", "stage": "conversion",
                             "reason": str(error)})
            else:
                if compile_result["exit_code"] != 0:
                    case.update({"status": "failed", "stage": "compile",
                                 "reason": "Shared harness compilation did not succeed"})
                else:
                    simulation = command_run(["vvp", executable, f"+imem={rows}",
                                              f"+tohost={args.tohost:08x}",
                                              f"+max_cycles={args.max_cycles}",
                                              f"+data_latency={args.data_latency}",
                                              f"+predict={args.predict}"],
                                             directory / "simulation.log", args.wall_timeout)
                    status, summaries = classify_simulation(simulation)
                    case.update({"status": status, "stage": "simulation",
                                 "summary": summaries,
                                 "simulation": without_output(simulation)})
        print(f"{case['name']}: {case['status']}", flush=True)
    report["counts"] = {status: sum(c["status"] == status for c in cases)
                        for status in ("passed", "failed", "conversion_failed", "timeout")}
    report["counts"]["total"] = len(cases)
    assert sum(report["counts"][key] for key in report["counts"] if key != "total") == len(files)
    passed = (bool(cases) and not report["errors"] and not provenance["errors"]
              and report["counts"]["passed"] == len(cases))
    report["status"] = "all_input_programs_reported_pass" if passed else "failed"
    report["finished_utc"] = datetime.now(timezone.utc).isoformat()
    text = json.dumps(report, indent=2) + "\n"
    report_path = run_dir / "results.json"
    report_path.write_text(text)
    (output / "results.json").write_text(text)
    print(f"Counts: {report['counts']}")
    print(f"Provenance: {provenance['status']}")
    print(f"Report: {report_path}")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
