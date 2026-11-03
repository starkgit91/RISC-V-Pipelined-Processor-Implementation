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
