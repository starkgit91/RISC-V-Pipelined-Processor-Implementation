#!/usr/bin/env python3
"""Keep the lesson's complete code blocks identical to the compiled source files."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BLOCKS = {
    "01": [
        ("Complete shared package", "rtl/rv32_ooo_pkg.sv"),
        ("Complete self-checking testbench", "tb/tb_block01.sv"),
    ],
    "02": [
        ("Complete instruction decoder", "rtl/rv32_decoder.sv"),
        ("Complete integer ALU", "rtl/rv32_alu.sv"),
        ("Complete branch comparator", "rtl/rv32_branch_compare.sv"),
        ("Complete self-checking testbench", "tb/tb_block02.sv"),
    ],
    "03": [
        ("Complete physical register file", "rtl/rv32_physical_regfile.sv"),
        ("Complete self-checking testbench", "tb/tb_block03.sv"),
    ],
    "04": [
        ("Complete rename, free-list, and readiness module", "rtl/rv32_rename.sv"),
        ("Complete self-checking testbench", "tb/tb_block04.sv"),
    ],
    "05": [
        ("Complete two-wide reorder buffer", "rtl/rv32_reorder_buffer.sv"),
        ("Complete architectural trace recorder", "rtl/rv32_trace_recorder.sv"),
        ("Complete self-checking testbench", "tb/tb_block05.sv"),
        ("Complete connected control/storage test", "tb/tb_block05_integration.sv"),
    ],
    "06": [
        ("Complete two-issue queue and scheduler", "rtl/rv32_issue_queue.sv"),
        ("Complete self-checking scheduler test", "tb/tb_block06.sv"),
        ("Complete connected scheduling test", "tb/tb_block06_integration.sv"),
    ],
    "07": [
        ("Complete integer/control/address execution", "rtl/rv32_integer_execute.sv"),
        ("Complete iterative multiply/divide execution", "rtl/rv32_muldiv_execute.sv"),
        ("Complete two-port execution cluster", "rtl/rv32_execution_cluster.sv"),
        ("Complete self-checking execution test", "tb/tb_block07.sv"),
        ("Complete connected out-of-order arithmetic test", "tb/tb_block07_integration.sv"),
    ],
}

for number, sources in BLOCKS.items():
    lesson = (ROOT / "docs" / f"block{number}_explanation.md").read_text()
    for title, filename in sources:
        lesson += f"\n## {title}\n\nSource: `{filename}`.\n\n```systemverilog\n"
        lesson += (ROOT / filename).read_text()
        lesson += "```\n"
    destination = ROOT / "docs" / f"block{number}_complete_guide.md"
    destination.write_text(lesson)
    print(f"Updated {destination.name} from the actual RTL and testbench.")
