// ACT4 platform hooks for tb_act4, not FPGA firmware or peripherals.
// Boot override pattern adapted from the official SERV ACT4 platform.
// Copyright (c) 2026, Harvey Mudd College
// SPDX-License-Identifier: Apache-2.0
#ifndef CUSTOM_RV32_ACT4_MODEL_MACROS_H
#define CUSTOM_RV32_ACT4_MODEL_MACROS_H

#define RVMODEL_DATA_SECTION

// Enable the common machine trap-handler interface. This macro does not claim
// certification: our override avoids optional CSRs that the core does not have.
#define STANDARD_SM_SUPPORTED

// Sail's reference-only CLINT is required by the ACT framework. Disarm it for
// signature generation so its initially pending timer cannot affect this ISA
// test platform, whose actual DUT IRQ pins remain low.
#ifdef SIGNATURE
  #define CUSTOM_ACT4_REFERENCE_IRQ_INIT \
    li t0, 0x40004000 ;                  \
    li t1, -1 ;                          \
    sw t1, 0(t0) ;                       \
    sw t1, 4(t0)
#else
  #define CUSTOM_ACT4_REFERENCE_IRQ_INIT
#endif

// Upstream's default boot writes all mhpmevent3..31 registers. Those optional
// registers trap on this core, so initialize only its implemented CSR subset.
// The prolog owns mtvec/mscratch after installation; do not overwrite them.
#define RVMODEL_BOOT_TO_MMODE           \
  rvtest_boot_to_mmode: ;               \
  csrw mie, zero ;                      \
  CUSTOM_ACT4_REFERENCE_IRQ_INIT ;      \
  csrw mip, zero ;                      \
  csrw mepc, zero ;                     \
  csrw mtval, zero ;                    \
  csrw mcause, zero ;                   \
  csrw mscratch, zero ;                 \
  RVTEST_TRAP_PROLOG M ;                \
  rvtest_boot_to_mmode_csr_init: ;       \
  li t0, 0x1800 ;                       \
  csrw mstatus, t0 ;                    \
  csrw mstatush, zero ;                 \
  csrw mcountinhibit, zero

// Exactly one word store reports the result. tb_act4 waits for its successful
// response and retirement before accepting it. Keep the configured address in
// sync with the runner's --tohost argument.
#define RVMODEL_HALT_PASS \
  li t0, 0x1000fff0 ;     \
  li t1, 1 ;              \
  sw t1, 0(t0) ;          \
  1: j 1b

#define RVMODEL_HALT_FAIL \
  li t0, 0x1000fff0 ;     \
  li t1, 2 ;              \
  sw t1, 0(t0) ;          \
  1: j 1b

// Console output is optional in this RAM/mailbox harness. The official test's
// pass/fail self-check remains active; no fake UART is accessed. Sail overrides
// this hook for reference signature generation.
#define RVMODEL_IO_WRITE_STR(_R1, _R2, _R3, _STR_PTR)

// Valid data/instruction accesses outside RAM fault in tb_act4. Keep this out
// of the mailbox range. Privileged fault tests are not selected in this pass.
#define RVMODEL_ACCESS_FAULT_ADDRESS 0x00010000

// Required by shared machine trap code, although this platform raises no IRQs.
#define RVMODEL_INTERRUPT_LATENCY 100
#define RVMODEL_TIMER_INT_SOON_DELAY 10000

#endif
