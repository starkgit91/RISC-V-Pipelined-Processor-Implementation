# Block 4: rename maps, the free list, and readiness

This block gives each register-writing instruction its own physical destination, while keeping
the architectural register names seen by the program. It implements speculative/committed maps,
physical allocation, readiness tracking, retirement updates, and the application of safe recovery.

The ROB controller, instruction scheduler, completion arbiter, and execution cancellation logic
are later blocks. Their validated events are modeled by this block's testbench.

## 1. Start with two writers of x5

Consider:

```asm
div x5, x1, x2
add x5, x3, x4
```

After reset, x5 maps to p5. Initially p32 through p63 are free. The allocator chooses the lowest
currently free physical identity, so the two writers receive p32 and p33 before either retires.

| Event | Speculative x5 map | Committed x5 map | New destination | Replaced physical identity |
|---|---|---|---|---|
| Reset | p5 | p5 | None | None |
| Dispatch DIV | p32 | p5 | p32 | p5 |
| Dispatch ADD | p33 | p5 | p33 | p32 |
| ADD completes first | p33 | p5 | No allocation | No release |
| DIV completes and retires | p33 | p32 | No allocation | p5 becomes free |
| ADD retires | p33 | p33 | No allocation | p32 becomes free |

Completion makes a value available to consumers, but does not change the committed map. The
younger ADD cannot overwrite DIV's p32 value, and retirement preserves the original instruction
order. This removes the WAW false dependency between their shared architectural destination.

Two writers alone demonstrate WAW, not WAR. To see WAR, insert an older reader before a writer:

```asm
add x6, x5, x1       # Captures the current source identity for x5.
add x5, x3, x4       # Gets a different destination; it cannot overwrite that source identity.
```

The older reader keeps its captured physical source even if the speculative x5 map changes.
The writer does not need to wait merely to protect the architectural name.

RAW is different: if a later instruction actually needs p33's value, it must wait until p33
is ready. Renaming preserves real dependencies; it does not remove them.

## 2. Hardware structures and where they connect

```text
                       ID: decoded register names + operation
                                      |
                                      v
                       speculative map: x-register -> p-register
                        |                                |
             capture source tags                  find old destination
                        |                                |
                        +------ free-list allocation ----+
                                      |
                           renamed instruction / ROB tag
                                      |
                         future ROB and issue queue

WB accepted completion ----------> readiness table
future ROB retirement -----------> committed map + old-destination release
safe recovery apply -------------> restore speculative map; rebuild free/readiness state

EX physical register file from block 3 stores the actual values separately.
```

Both maps have 32 entries of six bits each. The free list and readiness table each have one
bit per physical identity. Combinational selection reads mappings and finds a free identity;
clocked state updates apply allocations, completions, retirement, and recovery.

The physical register file is not copied when a mapping changes. The map chooses which stored
version a register name refers to. Shared package types `phys_mask_t` and `phys_count_t` describe
the mask and free-count connections.

## 3. Interface and ownership

| Port/group | Producer | Consumer | Meaning |
|---|---|---|---|
| `decoded_i` | Decoder | Rename | Architectural sources/destination, source-use flags, operation, and fault metadata |
| `dispatch_rob_tag_i` | Future ROB allocator | Renamed record | Identity reserved for this instruction when dispatch fires |
| `dispatch_valid_i` | ID/fetch holding register | Rename | There is a live instruction waiting to dispatch |
| `dispatch_resources_ready_i` | Future dispatch controller | Rename | All required ROB/issue/memory resources are available |
| `dispatch_ready_o` | Rename | Dispatch controller | This instruction can be accepted, including any physical destination |
| `renamed_o` | Rename | Future ROB/issue queue | Original operation, captured physical sources, new destination, and old destination |
| `src1_ready_o`, `src2_ready_o` | Readiness lookup/bypass | New issue entry | Whether the captured sources are available now |
| `completion_valid_i`, `completion_i` | Future completion arbiter | Readiness table | One accepted, live, ownership-validated result |
| `retire_valid_i`, `retire_uop_i` | Future ROB retirement | Committed map/free list | One successfully retiring instruction and its rename history |
| `recover_i` | Future recovery controller | Map/free/readiness state | Apply recovery after cancellation and result draining are complete |
| `recover_done_o` | Rename state register | Recovery controller | The recovery application has reached a clock edge |
| `committed_query_arch_i` / `committed_query_phys_o` | Debug/test query | Committed-map lookup | Read an architectural name's current committed physical identity |
| `free_mask_o`, `ready_mask_o`, `free_count_o` | Current state | Scheduling/control/debug | Registered availability state and its free-entry count |

The interface uses one atomic dispatch decision:

```text
dispatch_fire = dispatch_valid_i AND dispatch_ready_o
```

The controller must use that same event to allocate the instruction's other resources. It must
not allocate a physical destination while the corresponding ROB/issue/memory allocation fails.
No change occurs just because a candidate record is visible at `renamed_o`.

`renamed_o` is a combinational preview, not an independently buffered output channel. Capture
it only on dispatch_fire. Its candidate physical destination can change while dispatch is
stalled, for example when retirement makes a lower-numbered identity available.

## 4. Read sources before replacing the destination

For `addi x5,x5,1`, the source must refer to the version that existed before this instruction.
Both physical source lookups use the current speculative map. Only at an accepted dispatch edge
does the map change to the new destination.

For unused sources the physical tag is p0 and readiness is true. Immediate bits cannot create
false dependencies. x0 always maps to p0 and never allocates a destination. The module defensively
clears the destination-write flag for x0 or a faulted instruction even if its caller supplies it.

A valid fault token still needs a ROB entry for precise trapping, but needs no destination or
source wait. Do not drop it as an empty slot just because its execution controls are disabled.

## 5. Free identities and readiness are different

| Physical identity state | Free bit | Ready bit | Meaning |
|---|---:|---:|---|
| p0 | 0 | 1 | Reserved constant zero |
| Initial p1..p31 | 0 | 1 | Mapped startup values, reset to zero by block 3 |
| Initial p32..p63 | 1 | 0 | Available for allocation |
| Newly allocated destination | 0 | 0 | Owned by a producer whose value is not ready |
| Completed live destination | 0 | 1 | Value is available to consumers |
| Released replaced destination | 1 | 0 | Can be allocated again |

The allocator scans current free bits and chooses the lowest available identity. It does not
reuse an identity released by retirement on the same edge; that identity becomes available on
the next cycle. This removes same-edge recycling ambiguity and makes the handoff easy to inspect.

The free count describes physical storage availability, not ROB capacity. With 64 physical
identities, 32 architectural mappings, and a 16-entry ROB, a normal full-core execution cannot
exhaust all 32 initially free destinations. The standalone allocator test can deliberately
allocate more outstanding writers to exercise exhaustion, while making clear that the full
core's ROB would have stopped dispatch sooner.

## 6. Same-cycle completion and readiness

Completion can make a newly dispatched consumer ready on the same edge:

```text
current map says x5 -> p32; registered ready[p32] is still 0
accepted completion carries p32
consumer source lookup returns p32
src_ready output bypasses to 1 now
on the edge, ready[p32] becomes 1
```

The physical-register bypass from block 3 supplies the matching value. Control readiness and
data forwarding must refer to the same accepted completion. Neither may use an unaccepted
execution-unit result waiting behind another completion.

The module ignores readiness updates for a faulted result, a result with no GPR write, p0,
or a currently free destination. This is not a substitute for ROB identity validation. A stale
completion targeting a newly reallocated live physical identity can only be rejected by the
owning ROB/generation checks and cancellation rules in later blocks.

The normal update order is completion-ready, retirement release, then new allocation clears
its destination's readiness. Source readiness uses current mappings before that destination change.

## 7. Retirement changes the committed map

The retirement input means the instruction has successfully reached its in-order architectural
boundary. The future ROB must enforce this and must not retire a faulting or incomplete instruction.

For a real destination write, the new physical identity becomes committed and the old identity
is released. At this point `old_pdst` must equal that architectural register's current committed
mapping, and `pdst` must be the instruction's allocated nonzero destination. The ROB controller
will check this contract; the rename module does not independently implement a second ROB.

The speculative map is not overwritten by an ordinary retirement. It might already point to
an even younger version. This is why the two writers of x5 can finish and retire correctly
without forgetting the younger mapping.

## 8. Recovery applies the committed state without clearing values

`recover_i` is an apply pulse after the controller has cancelled younger execution, drained held
completions, and made resource reuse safe. This module does not perform that cancellation itself.

On the apply edge:

1. Incorporate any simultaneous successful retirement into the committed map.
2. Copy that updated committed map to the speculative map.
3. Mark all physical identities free except those present in the committed map.
4. Mark committed identities ready and all other identities not ready.
5. Preserve p0 as reserved/ready and acknowledge the application through recover_done_o.

Including retirement first matters for a mispredicted JAL/JALR. Its link register must commit
before younger work is discarded. Restoring yesterday's map would lose the valid link write.

Recovery blocks dispatch. If recover_i is held rather than pulsed, restoration repeats and
recover_done_o remains asserted after each edge; the future controller will use a single pulse.
There is no physical-register reset on recovery, so committed values remain intact.

## 9. Testing and waveform guidance

From the project directory:

```sh
python3 scripts/run_checks.py --block 4
python3 scripts/run_checks.py --block all
```

For an optional waveform, use `--block 4 --waves`. Reports and source hashes are saved in
`reports/block04_results.json` and `reports/block04_simulation.log`; the waveform is under
`build/block04/block04.vcd`.

The tests exercise two writers of x5, self-read/write source ordering, unused sources, readiness
bypass, blocked atomic allocation, exhaustion at allocator-unit level, retirement/freeing,
simultaneous events, safe-recovery application, and reset. Check exact coverage against the
testbench source and the recorded result; this is not whole-core out-of-order verification.

The testbench controls clock pulses explicitly so it can query all mappings between update edges.
This is a sequential-logic functional test, not a 50 MHz timing experiment. The 768 mixed
transactions use a small software queue to supply retirement in valid program order; that queue
is a test driver, not a delivered hardware ROB.

Useful waveform signals are dispatch_valid_i/ready_o, renamed_o's source/destination tags,
free_mask_o/count, ready_mask_o, src1/2_ready_o, retirement inputs, recover_i/done, and the
committed query for x5. During the two-writer example, observe different destination tags,
the newest source mapping, and committed x5 staying unchanged until retirement.

For XSim on Linux after sourcing Vivado's settings file:

```sh
xvlog --sv rtl/rv32_ooo_pkg.sv rtl/rv32_rename.sv tb/tb_block04.sv
xelab tb_block04 -s block04_sim
xsim block04_sim --runall
```

Icarus may report that constant selects in always_comb are sensitized to the whole vector.
This is a simulation sensitivity diagnostic; full messages remain in the log. Vivado resource
mapping, timing, and bitstream results remain unverified until run on your Linux installation.

## 10. Interview questions and model answers

**Does renaming solve every dependency?**
No. It removes false WAR/WAW dependencies caused by reusing register names. RAW dependencies
remain and are enforced using producer identities and readiness.

**Why retain both speculative and committed maps?**
The speculative map follows dispatched instructions; the committed map records architecturally
accepted versions and provides a safe recovery point.

**Why save an instruction's old physical destination?**
It identifies the version replaced at retirement. Freeing it at execution completion would be
too early because older work or committed state may still require it.

**Why not update the speculative map from every retirement?**
It may already contain a younger mapping. Overwriting it would lose that speculative dependency chain.

**Why is a zero value not the same as a ready value?**
The bits might be reset or left over from an earlier owner. Only the producer-completion/control
state establishes that this instruction's value is available.

**Why rebuild the free list from committed mappings on recovery?**
All younger ownership has been cancelled. The committed mappings identify the physical versions
that must remain live; the rest can be reused after the cancellation barrier.

**Can a source and destination use the same architectural name?**
Yes. Read the previous source mapping first, then allocate/update the destination mapping.

**What remains for a real out-of-order demonstration?**
The ROB, issue scheduler, overlapping execution units, completion arbitration, and integrated
recovery must be connected and tested. This block implements the required naming/readiness state.

## 11. References

- [BOOM rename-stage documentation](https://docs.boom-core.org/en/latest/sections/rename-stage.html)
- [BOOM reorder buffer and retirement](https://docs.boom-core.org/en/latest/sections/reorder-buffer.html)
- [lowRISC coding style](https://github.com/lowRISC/style-guides/blob/master/VerilogCodingStyle.md)

This is original project source. Its retirement-based recovery is intentionally conservative;
it does not implement BOOM's per-branch checkpoint mechanism.
