/* Compact derivative of the official SERV ACT4 linker layout.
 * Copyright (c) 2026, Harvey Mudd College
 * SPDX-License-Identifier: Apache-2.0
 * Every loadable byte, including reference-signature code/data, must fit RAM.
 */
OUTPUT_ARCH("riscv")
ENTRY(rvtest_entry_point)

RAM_ORIGIN = 0x00000000;
RAM_LENGTH = 0x00010000;
NUM_HARTS = 1;
STACK_SIZE = 0x1000;
PROVIDE(__stack_size = STACK_SIZE);
PROVIDE(__num_harts = NUM_HARTS);

MEMORY
{
  ram (rwx) : ORIGIN = RAM_ORIGIN, LENGTH = RAM_LENGTH
}

/* One physical image prevents separate ELF segments from silently overlapping.
 * The processor has no memory protection; its RAM is readable/writable/executable.
 */
PHDRS
{
  image PT_LOAD FLAGS(7);
}

SECTIONS
{
  .text.init ORIGIN(ram) : { KEEP(*(.text.init)) } > ram :image
  .text.rvtest ALIGN(4) : { *(.text.rvtest) *(.text.rvtest.*) } > ram :image
  .rodata ALIGN(16) : {
    *(.rodata) *(.rodata.*) *(.srodata) *(.srodata.*)
  } > ram :image
  .data ALIGN(16) : {
    *(.data) *(.data.*) *(.sdata) *(.sdata.*)
    /* Sail replaces RVMODEL_DATA_SECTION with its RAM-based HTIF symbols. */
    *(.tohost) *(.fromhost)
  } > ram :image
  .bss ALIGN(16) (NOLOAD) : {
    __bss_start = .;
    *(.sbss) *(.sbss.*) *(.bss) *(.bss.*) *(COMMON)
    . = ALIGN(16);
    __bss_end = .;
  } > ram :image
  .stack ALIGN(16) (NOLOAD) : {
    __stack_bottom = .;
    . += __stack_size * __num_harts;
    __stack_top = .;
  } > ram :image
  .text.rvmodel ALIGN(16) : {
    *(.text.rvmodel) *(.text.rvmodel.*) *(.text) *(.text.*)
  } > ram :image
  _end = ALIGN(16);
  ASSERT(rvtest_entry_point == RAM_ORIGIN, "ACT entry must match reset PC zero")
  ASSERT(_end <= ORIGIN(ram) + LENGTH(ram),
         "ACT executable exceeds the fixed 64 KiB RAM; split tests, do not enlarge RAM")
  /DISCARD/ : { *(.comment) *(.note.GNU-stack) *(.riscv.attributes) }
}
