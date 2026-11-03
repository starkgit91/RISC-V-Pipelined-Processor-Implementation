#!/usr/bin/env python3
"""Run the standalone ELF harness, or verify its PASS/FAIL/timeout paths.

The selftest uses locally encoded instructions, not official ACT4 test programs.
For an official suite, generate matching self-checking ELFs and retain their
configuration/provenance as described in docs/act4_verification.md.
"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def run(command, log_path, timeout):
    try:
        result = subprocess.run([str(item) for item in command], cwd=ROOT, text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                timeout=timeout, check=False)
        output, code = result.stdout, result.returncode
    except subprocess.TimeoutExpired as error:
        output = error.stdout or b""
        if isinstance(output, bytes):
            output = output.decode(errors="replace")
        output += "\nHOST_TIMEOUT: simulation exceeded the wall-clock limit\n"
        code = -1
    log_path.write_text(output)
    return {"command": [str(item) for item in command], "exit_code": code,
            "log": str(log_path), "output": output}


def minimal_elf(tohost, memory_test=False):
    words = [((tohost + 0x800) & 0xFFFFF000) | 0xB7,
             ((tohost & 0xFFF) << 20) | 0x8093]
    if memory_test:
        # A RAM word store, byte overwrite, and two unsigned byte reads must
        # preserve the untouched low byte. Both comparisons branch to FAIL.
        words += [0x10000113, 0x12300193, 0x00312023, 0x07F00213,
                  0x004100A3, 0x00014303, 0x02300393, 0,
                  0x00114303, 0x07F00393, 0,
                  0x00100113, 0x0020A023, 0x0000006F,
                  0x00200113, 0x0020A023, 0x0000006F]
        for branch in (9, 12):
            distance = 4 * (16 - branch)
            words[branch] = ((distance >> 12) & 1) << 31 \
                | ((distance >> 5) & 63) << 25 | 7 << 20 | 6 << 15 | 1 << 12 \
                | ((distance >> 1) & 15) << 8 | ((distance >> 11) & 1) << 7 | 0x63
    else:
        words += [0x00100113, 0x0020A023, 0x0000006F]
    program = struct.pack(f"<{len(words)}I", *words)
    ident = b"\x7fELF" + bytes((1, 1, 1)) + bytes(9)
    header = struct.pack("<HHIIIIIHHHHHH", 2, 243, 1, 0, 52, 0, 0,
                         52, 32, 1, 0, 0, 0)
    segment = struct.pack("<IIIIIIII", 1, 84, 0, 0, len(program), len(program), 5, 1)
    return ident + header + segment + program


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--selftest", action="store_true")
    mode.add_argument("--elf", type=Path)
    mode.add_argument("--imem", type=Path, help="already converted 64-bit-row hex")
    parser.add_argument("--tohost", type=lambda value: int(value, 0), default=0x1000FFF0)
    parser.add_argument("--max-cycles", type=int, default=1000000)
    parser.add_argument("--data-latency", type=int, default=2)
    parser.add_argument("--predict", type=int, choices=(0, 1), default=1)
    parser.add_argument("--wall-timeout", type=int, default=120)
    parser.add_argument("--vcd", type=Path)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "build/act4_harness")
    args = parser.parse_args()
    if not (0x10000 <= args.tohost <= 0xFFFFFFFC and args.tohost % 4 == 0):
        parser.error("--tohost must be a word-aligned 32-bit address outside RAM")
    if args.max_cycles <= 0 or args.data_latency < 0 or args.wall_timeout <= 0:
        parser.error("cycle limits must be positive and latency must be nonnegative")
    if not shutil.which("iverilog") or not shutil.which("vvp"):
        parser.error("Icarus Verilog and vvp must be installed")
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    report_path = output / "results.json"
    report_path.unlink(missing_ok=True)
    package = ROOT / "rtl/rv32_ooo_pkg.sv"
    sources = [package] + sorted(p for p in (ROOT / "rtl").glob("*.sv") if p != package)
    sources.append(ROOT / "tb/tb_act4.sv")
    executable = output / "tb_act4.vvp"
    compile_result = run(["iverilog", "-g2012", "-s", "tb_act4", "-o", executable]
                         + sources, output / "compile.log", 60)
    report = {"scope": "harness_selftest" if args.selftest else "input_program_execution",
              "certification_claim": False, "official_act4_elfs_executed": 0,
              "official_act4_status": "not_run" if args.selftest else "provenance_not_verified",
              "source_sha256": {str(p.relative_to(ROOT)): digest(p) for p in sources},
              "compile_exit_code": compile_result["exit_code"], "cases": []}
    if compile_result["exit_code"]:
        print(compile_result["output"][-4000:])
    else:
        def execute(name, plusargs, expected="PASS", expected_stores=1):
            result = run(["vvp", executable] + plusargs, output / f"{name}.log",
                         args.wall_timeout)
            success_exit = result["exit_code"] == 0
            marker = f"ACT4_HARNESS {expected}"
            expected_exit = success_exit if expected == "PASS" else not success_exit
            okay = marker in result["output"] and expected_exit
            if expected == "PASS" and args.selftest:
                okay = okay and f"stores={expected_stores}/{expected_stores}" in result["output"]
            summary = re.findall(r"ACT4_HARNESS (?:PASS|FAIL|TIMEOUT|CONFIG)[^\n]*",
                                 result["output"])
            report["cases"].append({"name": name, "expected": expected,
                                     "matched_expectation": okay,
                                     "exit_code": result["exit_code"],
                                     "summary": summary, "log": result["log"],
                                     "command": result["command"]})
            print(f"{name}: {'matched' if okay else 'FAILED'} expected {expected}")

        def convert(elf, prefix):
            rows = output / f"{prefix}.rows.hex"
            conversion = run([sys.executable, ROOT / "scripts/act4_elf_to_mem.py", elf,
                              "--words", output / f"{prefix}.words.hex", "--rows", rows,
                              "--report", output / f"{prefix}.load.json"],
                             output / f"{prefix}.conversion.log", 30)
            if conversion["exit_code"]:
                report["status"] = "conversion_failed"
                report["conversion"] = conversion
                report_path.write_text(json.dumps(report, indent=2) + "\n")
                parser.exit(1, conversion["output"])
            return rows

        if args.selftest:
            execute("builtin_pass", ["+selftest", "+max_cycles=1000"])
            execute("custom_address_slow_no_prediction", ["+selftest", "+max_cycles=1000",
                    "+tohost=20000000", "+data_latency=11", "+predict=0"])
            execute("builtin_fail", ["+selftest", "+selftest_code=2", "+max_cycles=1000"],
                    expected="FAIL")
            execute("zero_is_not_pass", ["+selftest", "+selftest_code=0", "+max_cycles=200"],
                    expected="TIMEOUT")
            elf = output / "synthetic_mailbox.elf"
            elf.write_bytes(minimal_elf(args.tohost))
            rows = convert(elf, "synthetic_mailbox")
            execute("synthetic_elf_image", [f"+imem={rows}", f"+tohost={args.tohost:08x}",
                    "+max_cycles=1000"])
            elf = output / "synthetic_ram.elf"
            elf.write_bytes(minimal_elf(args.tohost, memory_test=True))
            rows = convert(elf, "synthetic_ram")
            execute("synthetic_ram_byte_lanes", [f"+imem={rows}",
                    f"+tohost={args.tohost:08x}", "+max_cycles=1000"], expected_stores=3)
            execute("missing_image_is_error", ["+max_cycles=100"], expected="CONFIG")
        else:
            image = args.imem.resolve() if args.imem else convert(args.elf.resolve(), "input")
            report["input_sha256"] = digest(args.elf.resolve() if args.elf else image)
            plusargs = [f"+imem={image}", f"+tohost={args.tohost:08x}",
                        f"+max_cycles={args.max_cycles}", f"+data_latency={args.data_latency}",
                        f"+predict={args.predict}"]
            if args.vcd:
                plusargs.append(f"+vcd={args.vcd.resolve()}")
            execute("program", plusargs)
    passed = (compile_result["exit_code"] == 0 and bool(report["cases"])
              and all(case["matched_expectation"] for case in report["cases"]))
    report["status"] = ("selftest_passed" if args.selftest else "program_reported_pass") \
                       if passed else "failed"
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    if args.selftest:
        report_copy = ROOT / "reports/act4_harness_results.json"
        report_copy.write_text(json.dumps(report, indent=2) + "\n")
        log = []
        for case in report["cases"]:
            log.append(f"Case {case['name']}: expected {case['expected']}\n")
            log.append(Path(case["log"]).read_text())
        (ROOT / "reports/act4_harness_simulation.log").write_text("\n".join(log))
    print(f"Report: {report_path}")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
