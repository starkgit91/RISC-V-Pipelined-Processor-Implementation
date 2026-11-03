#!/usr/bin/env python3
"""Keep the lesson's complete code blocks identical to the compiled source files."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BLOCKS = {'01': [('Complete shared package', 'rtl/rv32_ooo_pkg.sv'),
        ('Complete self-checking testbench', 'tb/tb_block01.sv')],
 '02': [('Complete instruction decoder', 'rtl/rv32_decoder.sv'),
        ('Complete integer ALU', 'rtl/rv32_alu.sv'),
        ('Complete branch comparator', 'rtl/rv32_branch_compare.sv'),
        ('Complete self-checking testbench', 'tb/tb_block02.sv')],
 '03': [('Complete physical register file', 'rtl/rv32_physical_regfile.sv'),
        ('Complete self-checking testbench', 'tb/tb_block03.sv')],
 '04': [('Complete rename, free-list, and readiness module', 'rtl/rv32_rename.sv'),
        ('Complete self-checking testbench', 'tb/tb_block04.sv')],
 '05': [('Complete two-wide reorder buffer', 'rtl/rv32_reorder_buffer.sv'),
        ('Complete architectural trace recorder', 'rtl/rv32_trace_recorder.sv'),
        ('Complete self-checking testbench', 'tb/tb_block05.sv'),
        ('Complete connected control/storage test', 'tb/tb_block05_integration.sv')],
 '06': [('Complete two-issue queue and scheduler', 'rtl/rv32_issue_queue.sv'),
        ('Complete self-checking scheduler test', 'tb/tb_block06.sv'),
        ('Complete connected scheduling test', 'tb/tb_block06_integration.sv')],
 '07': [('Complete integer/control/address execution', 'rtl/rv32_integer_execute.sv'),
        ('Complete iterative multiply/divide execution', 'rtl/rv32_muldiv_execute.sv'),
        ('Complete two-port execution cluster', 'rtl/rv32_execution_cluster.sv'),
        ('Complete self-checking execution test', 'tb/tb_block07.sv'),
        ('Complete connected out-of-order arithmetic test', 'tb/tb_block07_integration.sv')],
 '08': [('Complete five-source completion arbiter', 'rtl/rv32_completion_arbiter.sv'),
        ('Complete accepted-value writeback wrapper', 'rtl/rv32_writeback.sv'),
        ('Complete arbitration/writeback testbench', 'tb/tb_block08.sv'),
        ('Complete connected production-writeback test', 'tb/tb_block08_integration.sv')],
 '09': [('Complete two-instruction fetch unit', 'rtl/rv32_fetch.sv'),
        ('Complete synchronous native instruction adapter', 'rtl/rv32_instruction_adapter.sv'),
        ('Complete fetch reference-model test', 'tb/tb_block09.sv'),
        ('Complete RAM-to-retirement integration test', 'tb/tb_block09_integration.sv'),
        ('Complete independent native-adapter test', 'tb/tb_block09_adapter.sv')],
 '10': [('Complete predictor', 'rtl/rv32_branch_predictor.sv'),
        ('Predictor tests', 'tb/tb_block10.sv'),
        ('Loop integration with test-only recovery', 'tb/tb_block10_integration.sv')],
 '11': [('Complete production recovery controller', 'rtl/rv32_recovery_controller.sv'),
        ('Complete recovery protocol testbench', 'tb/tb_block11.sv')],
 '12': [('Complete ordered LSU', 'rtl/rv32_load_store_unit.sv'), ('LSU tests', 'tb/tb_block12.sv')],
 '13': [('Complete CSR/system unit', 'rtl/rv32_csr_file.sv'), ('CSR tests', 'tb/tb_block13.sv')],
 '14': [('Complete AXI master', 'rtl/rv32_axi_lite_master.sv'),
        ('Complete machine timer', 'rtl/rv32_machine_timer.sv'),
        ('AXI/timer tests', 'tb/tb_block14.sv')],
 '15': [('Complete assembled two-issue core', 'rtl/rv32_ooo_core.sv'),
        ('Complete portable AXI wrapper', 'rtl/rv32_ooo_axi_top.sv'),
        ('Complete architectural reference testbench', 'tb/tb_block15.sv'),
        ('Complete precise-trap and interrupt testbench', 'tb/tb_core_traps.sv'),
        ('Complete AXI-connected processor testbench', 'tb/tb_core_axi.sv')]}

for number, sources in BLOCKS.items():
    lesson = (ROOT / "docs" / f"block{number}_explanation.md").read_text()
    for title, filename in sources:
        lesson += f"\n## {title}\n\nSource: `{filename}`.\n\n```systemverilog\n"
        lesson += (ROOT / filename).read_text()
        lesson += "```\n"
    destination = ROOT / "docs" / f"block{number}_complete_guide.md"
    destination.write_text(lesson)
    print(f"Updated {destination.name} from the actual RTL and testbench.")
