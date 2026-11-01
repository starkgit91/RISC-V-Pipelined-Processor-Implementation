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
        "scope": "four PRF reads, two writes, p0 protection, dual completion bypass",
        "not_verified": [
            "rename allocation or readiness tracking", "ROB tag validation or completion arbitration",
            "committed register maps or retirement", "FPGA resource mapping or timing",
        ],
    },
    "4": {
        "top": "tb_block04",
        "sources": ["rtl/rv32_ooo_pkg.sv", "rtl/rv32_rename.sv", "tb/tb_block04.sv"],
        "scope": "two-wide rename, intra-pair dependencies, dual completion/retirement, recovery",
        "not_verified": [
            "ROB controller and issue scheduling", "completion ownership validation",
            "execution cancellation or recovery quiescence detection",
            "physical result values or complete processor execution",
        ],
    },
    "5": {
        "top": "tb_block05",
        "sources": [
            "rtl/rv32_ooo_pkg.sv", "rtl/rv32_reorder_buffer.sv",
            "rtl/rv32_trace_recorder.sv", "tb/tb_block05.sv",
        ],
        "scope": "two-wide ROB ownership, ordered retirement/traps, recovery lock, registered trace",
        "not_verified": [
            "actual DIV/ADD execution or issue scheduling", "CSR/interrupt state updates",
            "execution cancellation and fetch redirection", "AXI memory effects",
        ],
        "extra_cases": [{
            "top": "tb_block05_integration",
            "summary": "BLOCK05_INTEGRATION",
            "scope": "connected rename/ROB/PRF/trace handshakes with injected completion values",
            "sources": [
                "rtl/rv32_ooo_pkg.sv", "rtl/rv32_rename.sv", "rtl/rv32_physical_regfile.sv",
                "rtl/rv32_reorder_buffer.sv", "rtl/rv32_trace_recorder.sv",
                "tb/tb_block05_integration.sv",
            ],
        }],
    },
    "6": {
        "top": "tb_block06",
        "sources": ["rtl/rv32_ooo_pkg.sv", "rtl/rv32_issue_queue.sv", "tb/tb_block06.sv"],
        "scope": "two-issue oldest-ready matching, source wakeup, resource and recovery gating",
        "not_verified": [
            "real execution units or completion arbitration", "execution cancellation",
            "full recovery coordination", "memory requests or CSR execution",
        ],
        "extra_cases": [{
            "top": "tb_block06_integration",
            "summary": "BLOCK06_INTEGRATION",
            "scope": "connected rename/ROB/PRF/IQ with modeled execution and injected completion",
            "sources": [
                "rtl/rv32_ooo_pkg.sv", "rtl/rv32_rename.sv", "rtl/rv32_physical_regfile.sv",
                "rtl/rv32_reorder_buffer.sv", "rtl/rv32_trace_recorder.sv",
                "rtl/rv32_issue_queue.sv",
                "tb/tb_block06_integration.sv",
            ],
        }],
    },
    "7": {
        "top": "tb_block07",
        "sources": [
            "rtl/rv32_ooo_pkg.sv", "rtl/rv32_alu.sv", "rtl/rv32_branch_compare.sv",
            "rtl/rv32_integer_execute.sv", "rtl/rv32_muldiv_execute.sv", "tb/tb_block07.sv",
        ],
        "scope": "integer/control/address execution, iterative RV32M, retained results, cancellation",
        "not_verified": [
            "production completion arbitration", "complete recovery controller",
            "memory transactions or CSR execution", "fetch or prediction lookup/training",
        ],
        "extra_cases": [{
            "top": "tb_block07_integration",
            "summary": "BLOCK07_INTEGRATION",
            "scope": "actual OoO arithmetic with connected rename/ROB/PRF/IQ/execution/trace",
            "sources": [
                "rtl/rv32_ooo_pkg.sv", "rtl/rv32_alu.sv", "rtl/rv32_branch_compare.sv",
                "rtl/rv32_rename.sv", "rtl/rv32_physical_regfile.sv",
                "rtl/rv32_reorder_buffer.sv", "rtl/rv32_trace_recorder.sv",
                "rtl/rv32_issue_queue.sv", "rtl/rv32_integer_execute.sv",
                "rtl/rv32_muldiv_execute.sv", "rtl/rv32_execution_cluster.sv",
                "tb/tb_block07_integration.sv",
            ],
        }],
    },
    "8": {
        "top": "tb_block08",
        "sources": [
            "rtl/rv32_ooo_pkg.sv", "rtl/rv32_completion_arbiter.sv", "rtl/rv32_writeback.sv",
            "tb/tb_block08.sv",
        ],
        "scope": "five-source/two-slot arbitration, retained outputs, accepted writeback, cancellation",
        "not_verified": [
            "memory/system execution engines", "complete recovery controller",
            "fetch or prediction lookup/training", "CSR/interrupt architectural state",
        ],
        "extra_cases": [{
            "top": "tb_block08_integration",
            "summary": "BLOCK08_INTEGRATION",
            "scope": "real execution with production completion arbitration and accepted writeback",
            "sources": [
                "rtl/rv32_ooo_pkg.sv", "rtl/rv32_alu.sv", "rtl/rv32_branch_compare.sv",
                "rtl/rv32_rename.sv", "rtl/rv32_physical_regfile.sv",
                "rtl/rv32_reorder_buffer.sv", "rtl/rv32_trace_recorder.sv",
                "rtl/rv32_issue_queue.sv", "rtl/rv32_integer_execute.sv",
                "rtl/rv32_muldiv_execute.sv", "rtl/rv32_execution_cluster.sv",
                "rtl/rv32_completion_arbiter.sv", "rtl/rv32_writeback.sv",
                "tb/tb_block08_integration.sv",
            ],
        }],
    },
    "9": {
        "top": "tb_block09",
        "sources": ["rtl/rv32_ooo_pkg.sv", "rtl/rv32_fetch.sv", "tb/tb_block09.sv"],
        "scope": "paired fetch, response credits, prefix buffering, prediction metadata, stale reads",
        "not_verified": [
            "BTB/PHT hardware or training", "complete recovery controller",
            "shared-port write collision detector", "AXI, memory instructions, CSRs, interrupts",
        ],
        "extra_cases": [{
            "top": "tb_block09_integration",
            "summary": "BLOCK09_INTEGRATION",
            "scope": "synchronous instruction RAM through dual decode to actual OoO retirement",
            "sources": [
                "rtl/rv32_ooo_pkg.sv", "rtl/rv32_fetch.sv", "rtl/rv32_instruction_adapter.sv",
                "rtl/rv32_decoder.sv", "rtl/rv32_alu.sv", "rtl/rv32_branch_compare.sv",
                "rtl/rv32_rename.sv", "rtl/rv32_physical_regfile.sv",
                "rtl/rv32_reorder_buffer.sv", "rtl/rv32_trace_recorder.sv",
                "rtl/rv32_issue_queue.sv", "rtl/rv32_integer_execute.sv",
                "rtl/rv32_muldiv_execute.sv", "rtl/rv32_execution_cluster.sv",
                "rtl/rv32_completion_arbiter.sv", "rtl/rv32_writeback.sv",
                "tb/tb_block09_integration.sv",
            ],
        }],
    },
}


def run_block(block, simulator_version, waves=False):
    spec = BLOCKS[block]
    label = f"block{int(block):02d}"
    build = ROOT / "build" / label
    reports = ROOT / "reports"
    build.mkdir(parents=True, exist_ok=True)
    reports.mkdir(parents=True, exist_ok=True)
    cases = [{"top": spec["top"], "sources": spec["sources"],
              "scope": spec["scope"], "summary": f"BLOCK{int(block):02d}"}]
    cases += spec.get("extra_cases", [])
    log_parts = []
    case_results = []
    source_files = set()
    try:
        for case in cases:
            source_files.update(case["sources"])
            commands = [
                ["iverilog", "-g2012", "-Wall", "-s", case["top"], "-o",
                 str(build / (case["top"] + ".vvp")), *case["sources"]],
                ["vvp", str(build / (case["top"] + ".vvp"))],
            ]
            if waves and block in {"3", "4", "5", "6", "7", "8", "9"}:
                commands[-1].append("+waves")
            for command in commands:
                result = subprocess.run(command, cwd=ROOT, text=True, capture_output=True, timeout=30)
                log_parts.append("$ " + " ".join(command) + "\n" + result.stdout + result.stderr)
                if result.returncode:
                    raise RuntimeError(f"{case['top']} failed with exit code {result.returncode}")
            pattern = rf"{case['summary']} SUMMARY: (\d+) checks, (\d+) failures"
            summary = re.search(pattern, log_parts[-1])
            if not summary or int(summary.group(2)):
                raise RuntimeError(f"A passing summary was not found for {case['top']}")
            case_results.append({"top": case["top"], "scope": case["scope"],
                                 "checks": int(summary.group(1)), "failures": 0})
        total_checks = sum(item["checks"] for item in case_results)
        evidence = {
            "block": int(block),
            "status": "passed",
            "architecture_target": "two_issue_RV32IM_Zicsr_Zifencei",
            "scope": spec["scope"],
            "checks": total_checks,
            "failures": 0,
            "cases": case_results,
            "simulator": simulator_version,
            "sources_sha256": {
                name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
                for name in sorted(source_files)
            },
            "not_verified": spec["not_verified"] + [
                "complete processor, AXI, interrupts, or firmware",
                "Vivado synthesis, implementation, or bitstream generation",
            ],
        }
        (reports / f"{label}_results.json").write_text(json.dumps(evidence, indent=2) + "\n")
        print(f"Block {block}: {total_checks} checks passed. Scope: {spec['scope']}.")
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
    parser.add_argument("--waves", action="store_true", help="Write optional clocked VCD waveforms")
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
