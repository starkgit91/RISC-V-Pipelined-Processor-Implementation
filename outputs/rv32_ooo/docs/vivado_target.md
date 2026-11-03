# Retained Vivado target specifications

These are the fixed implementation targets selected for the RTL project. Firmware, FPGA/IP
integration, board constraints, and bitstream generation were removed from the requested scope.
The table specifies the intended environment; it is not an implementation report.

| Item | Selected target |
|---|---|
| HDL | Synthesizable SystemVerilog `.sv` |
| Tool environment | Vivado 2026.1 on Linux |
| FPGA family/part | Artix-7 `xc7a35tcpg236-1` |
| Reference board | Basys 3 |
| Reference input clock | 100 MHz |
| Supplied core/AXI clock target | 50 MHz, one shared clock domain |
| Reset convention | Active-low asynchronous reset at the custom RTL boundary |
| Program/data RAM capacity | 64 KiB shared storage |
| Data width | 32-bit words with four byte write strobes |
| Native instruction width | 64-bit row, holding two 32-bit instructions |
| Native instruction depth/address | 8,192 rows / 13-bit RAM-relative row index |
| Native read latency | One synchronous read stage; additional BRAM output stages disabled |
| CPU data transport | One-outstanding AXI4-Lite master |
| Interrupt inputs | Synchronous external, software, and timer pending levels |

The RTL expects the surrounding system to supply its actual clock/reset, memory endpoints,
address routing, and interrupt sources. The portable AXI top exposes a native instruction-memory
port and a data AXI-Lite port. It contains no clock-generation or vendor block-design logic.

The native instruction adapter includes a read-hold input for a surrounding shared-memory
collision detector. The 32-bit data and 64-bit instruction views must describe the same
little-endian storage. Address conversion and memory latency must match the documented native
port contract; enabling additional memory pipeline stages requires corresponding adaptation.

The address plan keeps RAM at 0x00000000–0x0000ffff and the machine timer region at
0x02000000–0x0200ffff. The earlier UART/GPIO locations, 0x10000000 and 0x10010000, remain reserved
integration addresses; vendor UART/GPIO IP and a production system fabric are not supplied.

No Vivado synthesis, placement/routing, static timing, DRC, FPGA resource-fit, or bitstream result
is claimed. The 50 MHz setting is an implementation target. The functional simulation results
are separate evidence and do not establish that target's timing closure.
