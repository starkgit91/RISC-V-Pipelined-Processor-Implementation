#!/usr/bin/env python3
"""Report local ACT4 prerequisites without installing or running the suite."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import platform
import plistlib
import shutil
import subprocess
import tempfile

TOOLS = ("riscv32-unknown-elf-gcc", "riscv64-unknown-elf-gcc", "clang", "ld.lld",
         "llvm-objcopy", "sail_riscv_sim", "riscv_sim_RV32", "mise", "uv", "ruby",
         "bundle", "make", "git", "iverilog", "vvp", "docker")
SOURCE = ".text\n.globl _start\n_start:\n addi x1,x0,7\n mul x2,x1,x1\n" \
         " csrr x3,mstatus\n fence.i\n"


def run(command, timeout=15):
    try:
        result = subprocess.run(command, text=True, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, timeout=timeout, check=False)
        return {"returncode": result.returncode, "output": result.stdout[:4000]}
    except (OSError, subprocess.TimeoutExpired) as error:
        return {"returncode": None, "output": str(error)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--compiler", help="explicit RISC-V GCC or clang executable")
    parser.add_argument("--act4-checkout", type=Path)
    args = parser.parse_args()
    tools = {name: shutil.which(name) for name in TOOLS}
    compiler = args.compiler or next((tools[name] for name in TOOLS[:3] if tools[name]), None)
    report = {"checked_utc": datetime.now(timezone.utc).isoformat(),
              "host": platform.platform(), "status": "act4_not_run",
              "tools": tools, "compiler": compiler,
              "suite_elfs_executed": 0, "certification_claim": False}
    if compiler:
        report["compiler_version"] = run([compiler, "--version"])
        with tempfile.TemporaryDirectory(prefix="act4-probe-") as directory:
            source = Path(directory) / "probe.S"
            target = Path(directory) / "probe.o"
            source.write_text(SOURCE)
            flags = (["--target=riscv32-unknown-elf"]
                     if "clang" in Path(compiler).name else [])
            common = ["-march=rv32im_zicsr_zifencei", "-mabi=ilp32"]
            result = run([compiler] + flags + common + ["-c", str(source), "-o", str(target)])
            result["object_produced"] = target.exists()
            result["command_template"] = [compiler] + flags + common + ["-c", "probe.S",
                                                                       "-o", "probe.o"]
            report["rv32_assembler_probe"] = result
    else:
        report["rv32_assembler_probe"] = {"returncode": None, "object_produced": False,
                                           "output": "No candidate compiler found"}
    if tools["docker"]:
        report["docker_server_probe"] = run([tools["docker"], "version", "--format",
                                               "{{.Server.Version}}"], timeout=5)
    docker_bundle = Path("/Applications/Docker.app/Contents")
    if (docker_bundle / "Info.plist").is_file():
        info = plistlib.loads((docker_bundle / "Info.plist").read_bytes())
        executable = docker_bundle / "MacOS" / info.get("CFBundleExecutable", "")
        report["docker_desktop_read_only_inspection"] = {
            "installed": True, "version": info.get("CFBundleShortVersionString"),
            "launch_executable": str(executable),
            "launch_executable_present": executable.is_file(),
            "start_attempted_by_this_probe": False,
            "successful_start_not_established_by_file_inspection": True,
        }
    if args.act4_checkout:
        checkout = args.act4_checkout.resolve()
        report["act4_checkout"] = str(checkout)
        report["act4_revision"] = run(["git", "-C", str(checkout), "rev-parse", "HEAD"])
        report["framework_files_present"] = all((checkout / name).is_file()
                                                 for name in ("README.md", "Makefile"))
    else:
        report["act4_checkout"] = None
    report["not_established"] = [
        "Matching UDB/Sail/platform configuration validated against the chosen ACT4 revision",
        "Compiler linker and reference model can build and run the applicable test suite",
        "Every applicable self-checking ELF fits the fixed 64 KiB RAM",
        "Complete upstream ACT4 platform configuration and console/interrupt hooks validated",
        "ACT4 execution or architectural certification",
    ]
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
