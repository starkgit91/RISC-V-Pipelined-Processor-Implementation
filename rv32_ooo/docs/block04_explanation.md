# Block 4, two-issue revision: rename a pair without losing program order

This is the active two-wide rename block. The previous single-instruction lesson, sources,
and reports remain in `reference/single_issue`.

## 1. Two writers of x5 in the same dispatch group

```asm
div x5, x1, x2       # Lane 0: older
add x5, x3, x4       # Lane 1: younger
```

Initially x5 maps to p5 and p32..p63 are free. Lane 0 allocates p32 and records old_pdst=p5.
Lane 1 observes that update, allocates p33, and records old_pdst=p32. The speculative map ends
at p33. The committed map stays p5 until retirement.

If both retire together in program order, committed x5 first becomes p32 and then p33 in the
same edge. The final mapping is p33; p5 and p32 are released. The retirement trace will still
record both instructions in order. This example concerns naming/state; the divider is not
implemented in this block.

## 2. Real and false dependencies inside the pair

```asm
add x5, x1, x2
add x6, x5, x3
```

Lane 1 must capture lane 0's new physical destination as its x5 source, with readiness false.
It cannot read the old x5 value, or become ready merely because that physical location contains
old bits. This is a true RAW dependency and still delays execution.

For two writers of x5, separate destinations remove WAW. For an older reader followed by a
writer, lane 0 retains its old physical source while lane 1 gets a different destination,
removing WAR. An instruction reading and writing its own architectural name captures sources
before replacing its destination mapping.

## 3. Circuit and data flow

```text
                current speculative map / current free list
                               |
                   lane 0 source and destination lookup
                               |
                    accepted lane 0 mapping bypass
                               |
                   lane 1 source and destination lookup
                               |
                   next speculative map / allocated tags

 two accepted completions -> readiness lookup and registered ready bits
 two ordered retirements  -> committed map, then old-destination releases
 safe recovery APPLY      -> copy final committed map and rebuild availability
```

Both map tables contain 32 six-bit physical identities. The 64-bit free and readiness masks
remain separate from the physical values stored in block 3. Two priority selections allocate
from current free entries, removing lane 0's accepted destination before choosing lane 1's.

## 4. Pair interface and acceptance

The decoded, renamed, completion, retirement, and ROB-tag connections use named lane0/lane1
packed pairs. For dispatch and retirement, lane 0 is older. Dispatch validity and successful
retirement use prefix masks: 00 means none, 01 means lane 0, 11 means both.

| Signal group | Width/type | Source → destination | Meaning |
|---|---|---|---|
| clk_i / rst_ni | 1 each | Common clock/reset → maps and masks | Rising-edge updates; asynchronous active-low reset |
| decoded_i | 528; decoded_pair_t | Two decoders → rename | Two operation records with architectural source/destination fields |
| dispatch_rob_tag_i | 24; two 12-bit tags | ROB allocator → rename | Each tag contains a four-bit slot and eight-bit generation |
| dispatch_valid_i | 2 | Fetch/dispatch holding queue → rename | Instructions offered in program order |
| dispatch_resources_ready_i | 2 | Global dispatch control → rename | Per-lane ROB/issue/memory capacity agreement |
| dispatch_ready_o / dispatch_fire_o | 2 each | Rename → atomic dispatcher/fetch | Available capacity and the accepted ordered prefix |
| renamed_o | 600; renamed_pair_t | Rename → ROB/IQ/LSU | Source tags and distinct new/old destination tags; each physical tag is six bits |
| src1_ready_o / src2_ready_o | 2 each | Rename ready lookup → IQ | Readiness of each lane's actual physical sources, including accepted-completion bypass |
| completion_valid_i / completion_i | 2 / 444; completion_pair_t | Validated writeback → readiness table | Up to two accepted results; mask 10 is legal here |
| retire_valid_i / retire_uop_i | 2 / 600; renamed_pair_t | ROB retirement → committed maps/free list | Successful retirements, oldest first; release replaced physical mappings |
| recover_i | 1 | Recovery APPLY → rename | Restore speculative ownership after cancellation/drain |
| recover_done_o | 1 | Rename → recovery controller | Registered acknowledgment of the restoration edge |
| free_mask_o / ready_mask_o | 64 each; phys_mask_t | Current rename state → status/control | One ownership/readiness bit per physical register |
| free_count_o | 7; phys_count_t | Free-mask count → status/control | Number of available physical identities, range 0–64 |
| committed_query_arch_i | 5; arch_reg_t | Debug/test query → committed map | Select architectural x0 through x31 |
| committed_query_phys_o | 6; phys_reg_t | Committed map → debug/test | Physical mapping for the selected committed register |
| protocol_error_o | 1 | Rename diagnostics → integration checks | Invalid dispatch/retirement prefix or duplicate ready-producing write |

Aggregate widths are packed wire widths in the fixed configuration, not a count of RAM blocks.
The paired record shapes are defined once in the shared package, so every consumer sees the
same source tags, rename history, and ROB ownership metadata.

Lane 1 transfers only if lane 0 transfers. If there is space for only the older instruction,
fire=01. The front end must retain the younger suffix and compact it into lane 0 for the next
cycle; it must not resend the already accepted older instruction.

The older ready signal advertises capacity independently of valid. An invalid 10 offer still
cannot transfer because valid0=0 prevents both fire0 and lane1 readiness. This distinction
avoids a combinational ready/valid loop when the global dispatcher joins several resources.

The candidate renamed records are combinational previews, not independent buffered channels.
Capture a lane only on its fire bit. Candidates may change while stalled as resources change.

## 5. Allocation, completion, and retirement ordering

Unused sources and x0 select p0, which is always ready and never free. A valid fault token
requires a ROB entry but no physical destination or source wait. The decoder's original PC,
prediction, and exception information are retained.

Readiness lookup includes both accepted live non-faulting completions. A destination newly
allocated by lane 0 is explicitly unready when lane 1 looks it up. A duplicate physical
completion is diagnosed and lane 0 has priority; the upstream arbiter must prevent it.

Only accepted validated completions can publish readiness. This module does not independently
check ROB generations, so the future ROB/completion controller must reject stale owners before
asserting completion_valid. A ready bit is not an architectural commit.

Retirement applies lane 0 and then lane 1. The old physical identity for each writer must match
its replaced committed mapping at that point in the prefix. The ROB/controller supplies legal
in-order retirement records. Ordinary retirement must not overwrite the speculative map, which
may already name younger values.

Allocation uses the current free pool rather than retirement releases from the same edge.
Newly released tags can be reused next cycle. With only one free tag, the older writer can
transfer while the younger writer waits. A nonwriter or fault can transfer without a free tag.
The testbench may exhaust the allocator alone; the final 16-entry ROB prevents that 32-writer
exhaustion scenario during normal whole-core operation.

## 6. Recovery

recover_i is a safe APPLY pulse after execution cancellation and result draining. It blocks
new dispatch, applies any simultaneous successful retirement prefix, then restores speculative
mappings from the resulting committed map. Free and ready masks are rebuilt from those mappings.
This preserves a retiring branch's link version, including ordered same-register retirements.

Physical register values are not cleared. Cancellation/ownership validation and safe reuse
are implemented in later controller blocks; a generation counter alone is not the barrier.

## 7. Tests and simulator use

```sh
python3 scripts/run_checks.py --block 4 --waves
```

Tests cover intra-pair RAW/WAW/WAR, self-read/write, partial acceptance and compaction,
simultaneous completion/retirement/allocation, readiness, exhaustion at allocator-unit level,
recovery, protocol diagnostics, and reset. Check the report for exact count and source hashes.
The controlled clock pulses allow independent state queries; they do not measure FPGA timing.

The optional waveform is `build/block04/block04.vcd`. Observe fire masks, both physical
destinations, lane 1's bypassed physical source, source-ready bits, and committed x5. The first
pair must not turn a RAW-dependent lane 1 instruction into an independent operation.

For Vivado XSim after sourcing the Linux installation's settings:

```sh
xvlog --sv rtl/rv32_ooo_pkg.sv rtl/rv32_rename.sv tb/tb_block04.sv
xelab tb_block04 -s block04_sim
xsim block04_sim --runall
```

The actual Vivado fit, timing, and bitstream remain unverified. Icarus may emit its known
always_comb whole-vector sensitivity notices; complete messages are retained in the log.

## 8. Interview questions

**Why is lane 0's mapping visible to lane 1 before the edge?** Program order still determines
which version each instruction reads, even when two instructions rename together.

**Why can't the two instructions simply rename independently?** That would miss within-pair
RAW and WAW relationships and could allocate the same physical destination twice.

**Can lane 1 transfer while lane 0 stalls?** No. Dispatch accepts a program-order prefix.
A leftover younger instruction is compacted into the older position next cycle.

**Why can completion mask10 be valid while retirement mask10 is invalid?** Results may arrive
independently; architectural retirement may not skip an older instruction in the same prefix.

**Do two writes to x5 need only one physical destination?** No. They represent different
versions. Both must remain distinct until ordering permits the old versions to be reclaimed.

**Does dual retirement change the final architectural meaning?** No. It applies two sequential
architectural updates in one edge, with lane 1 observing lane 0's committed update.

The complete active RTL and testbench follow. Historical single-issue code is retained only
as a reference and is not compiled by the active test runner.
