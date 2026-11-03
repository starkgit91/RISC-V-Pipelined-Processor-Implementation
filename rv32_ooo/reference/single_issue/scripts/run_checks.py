#!/usr/bin/env python3
"""Run checks for implemented blocks; never imply unimplemented hardware was tested."""

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]

BLOCKS = {
    "1": {
        "top": "tb_block01",
        "sources": ["rtl/rv32_ooo_pkg.sv", "tb/tb_block01.sv"],
        "scope": "configuration constants and packed instruction/queue record contracts",
        "not_verified": ["instruction decoding or execution", "renaming or scheduling"],
    },
    "2": {
        "top": "tb_block02",
        "sources": [
            "rtl/rv32_ooo_pkg.sv", "rtl/rv32_decoder.sv", "rtl/rv32_alu.sv",
            "rtl/rv32_branch_compare.sv", "tb/tb_block02.sv",
        ],
        "scope": "instruction decode, immediate construction, integer ALU, and branch conditions",
        "not_verified": [
            "multiply/divide execution", "jump execution or prediction recovery",
            "physical registers, renaming, scheduling, or retirement",
            "memory transactions, CSR execution, or trap delivery",
        ],
    },
    "3": {
        "top": "tb_block03",
        "sources": ["rtl/rv32_ooo_pkg.sv", "rtl/rv32_physical_regfile.sv", "tb/tb_block03.sv"],
        "scope": "clocked physical register storage, two reads, x0 protection, completion bypass",
        "not_verified": [
            "rename allocation or readiness tracking", "ROB tag validation or completion arbitration",
            "committed register maps or retirement", "FPGA resource mapping or timing",
        ],
    },
    "4": {
        "top": "tb_block04",
        "sources": ["rtl/rv32_ooo_pkg.sv", "rtl/rv32_rename.sv", "tb/tb_block04.sv"],
        "scope": "rename maps, physical allocation, readiness, retirement updates, recovery apply",
        "not_verified": [
            "ROB controller and issue scheduling", "completion ownership validation",
            "execution cancellation or recovery quiescence detection",
            "physical result values or complete processor execution",
        ],
    },
}


def run_block(block, simulator_version, waves=False):
    spec = BLOCKS[block]
    label = f"block{int(block):02d}"
    build = ROOT / "build" / label
    reports = ROOT / "reports"
    build.mkdir(parents=True, exist_ok=True)
    reports.mkdir(parents=True, exist_ok=True)
    commands = [
        ["iverilog", "-g2012", "-Wall", "-s", spec["top"], "-o",
         str(build / (spec["top"] + ".vvp")), *spec["sources"]],
        ["vvp", str(build / (spec["top"] + ".vvp"))],
    ]
    if waves and block in {"3", "4"}:
        commands[-1].append("+waves")
    log_parts = []
    try:
        for command in commands:
            result = subprocess.run(command, cwd=ROOT, text=True, capture_output=True, timeout=30)
            log_parts.append("$ " + " ".join(command) + "\n" + result.stdout + result.stderr)
            if result.returncode:
                raise RuntimeError(f"Command failed with exit code {result.returncode}")
        pattern = rf"BLOCK{int(block):02d} SUMMARY: (\d+) checks, (\d+) failures"
        summary = re.search(pattern, log_parts[-1])
        if not summary or int(summary.group(2)):
            raise RuntimeError("A passing testbench summary was not found")
        evidence = {
            "block": int(block),
            "status": "passed",
            "scope": spec["scope"],
            "checks": int(summary.group(1)),
            "failures": int(summary.group(2)),
            "simulator": simulator_version,
            "sources_sha256": {
                name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
                for name in spec["sources"]
            },
            "not_verified": spec["not_verified"] + [
                "complete processor, AXI, interrupts, or firmware",
                "Vivado synthesis, implementation, or bitstream generation",
            ],
        }
        (reports / f"{label}_results.json").write_text(json.dumps(evidence, indent=2) + "\n")
        print(f"Block {block}: {summary.group(1)} checks passed. Scope: {spec['scope']}.")
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
        # Replace old success evidence so a failed rerun cannot leave a false green result.
        (reports / f"{label}_results.json").write_text(
            json.dumps({"block": int(block), "status": "failed", "error": str(exc)}, indent=2)
            + "\n"
        )
        print(f"Block {block} failed: {exc}", file=sys.stderr)
        return 1
    finally:
        (reports / f"{label}_simulation.log").write_text("\n".join(log_parts))
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--block", choices=[*BLOCKS, "all"], default="all")
    parser.add_argument("--waves", action="store_true", help="Write optional block 3/4 VCD waveforms")
    args = parser.parse_args()
    for executable in ("iverilog", "vvp"):
        if shutil.which(executable) is None:
            parser.error(f"{executable} is required; use the README's Vivado commands instead.")

    version = subprocess.run(["iverilog", "-V"], text=True, capture_output=True, timeout=10)
    simulator_version = version.stdout.splitlines()[0]
    blocks = list(BLOCKS) if args.block == "all" else [args.block]
    for block in blocks:
        if run_block(block, simulator_version, args.waves):
            return 1
    print("These are block-level checks; complete processor and Vivado checks remain pending.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
