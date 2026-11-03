#!/usr/bin/env python3
"""Convert a matching ACT4 ELF to this core's fixed, little-endian RAM image.

This is an ELF loader, not an ACT4 runner. It never interprets a conversion as a
test pass. No external packages or RISC-V objcopy installation are required.
"""

import argparse
import hashlib
import json
from pathlib import Path
import struct

RAM_BYTES = 64 * 1024
RESET_PC = 0


class ElfError(ValueError):
    """The executable cannot be represented by this core's memory map."""


def load_elf(data):
    """Return the complete byte-addressed RAM and validated load metadata."""
    if len(data) < 52:
        raise ElfError("truncated ELF32 header")
    ident = data[:16]
    if ident[:4] != b"\x7fELF" or ident[4:7] != bytes((1, 1, 1)):
        raise ElfError("requires ELF32, little endian, ELF version 1")
    header = struct.unpack_from("<HHIIIIIHHHHHH", data, 16)
    kind, machine, version, entry, phoff, _, flags, ehsize, phsize, phnum, *_ = header
    if kind != 2 or machine != 243 or version != 1:
        raise ElfError("requires a linked ET_EXEC RISC-V executable")
    if flags != 0:
        raise ElfError("requires RV32 integer ABI without RVC/RVE/float/TSO ELF flags")
    if ehsize != 52 or phsize != 32 or phnum == 0 or phnum == 0xFFFF:
        raise ElfError("unsupported or missing ELF32 program-header table")
    if phoff < 52 or phoff + phnum * phsize > len(data):
        raise ElfError("truncated program-header table")
    if entry != RESET_PC:
        raise ElfError("entry must match the fixed reset PC 0x00000000")

    ram = bytearray(RAM_BYTES)
    segments = []
    for index in range(phnum):
        fields = struct.unpack_from("<IIIIIIII", data, phoff + index * phsize)
        ptype, offset, vaddr, paddr, filesz, memsz, permissions, alignment = fields
        if ptype != 1:
            continue
        if filesz > memsz or offset + filesz > len(data):
            raise ElfError(f"segment {index}: invalid file extent")
        if vaddr != paddr:
            raise ElfError(f"segment {index}: virtual and physical addresses differ; no MMU")
        if paddr > RAM_BYTES or memsz > RAM_BYTES - paddr:
            raise ElfError(f"segment {index}: outside the fixed 64 KiB RAM")
        if alignment not in (0, 1):
            if alignment & (alignment - 1) or offset % alignment != vaddr % alignment:
                raise ElfError(f"segment {index}: invalid ELF alignment")
        if not memsz:
            continue
        end = paddr + memsz
        if any(paddr < item["end"] and item["start"] < end for item in segments):
            raise ElfError(f"segment {index}: overlapping load segments are unsupported")
        ram[paddr:paddr + filesz] = data[offset:offset + filesz]
        segments.append({"start": paddr, "end": end, "file_bytes": filesz,
                         "zero_fill_bytes": memsz - filesz,
                         "executable": bool(permissions & 1)})
    if not any(item["start"] <= entry < item["end"] and item["executable"]
               for item in segments):
        raise ElfError("reset PC is not covered by an executable PT_LOAD segment")
    return ram, {"entry": entry, "ram_bytes": RAM_BYTES, "segments": segments}


def memory_text(ram, lane_bytes):
    """A line's least significant byte has the lowest byte address."""
    return "".join(f"{int.from_bytes(ram[n:n + lane_bytes], 'little'):0{lane_bytes * 2}x}\n"
                   for n in range(0, len(ram), lane_bytes))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("elf", type=Path)
    parser.add_argument("--words", type=Path, required=True,
                        help="16384 lines for a SystemVerilog 32-bit word array")
    parser.add_argument("--rows", type=Path,
                        help="optional 8192 lines for a 64-bit instruction-row array")
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    try:
        source = args.elf.read_bytes()
        ram, report = load_elf(source)
    except (OSError, ElfError) as error:
        parser.exit(1, f"ELF rejected: {error}\n")
    destinations = [p.resolve() for p in (args.words, args.rows, args.report) if p]
    if args.elf.resolve() in destinations or len(destinations) != len(set(destinations)):
        parser.exit(1, "input and output files must all be distinct\n")
    report.update({"status": "converted_not_executed", "source": str(args.elf),
                   "elf_sha256": hashlib.sha256(source).hexdigest(), "outputs": {}})
    for path, width in ((args.words, 4), (args.rows, 8)):
        if path:
            path.parent.mkdir(parents=True, exist_ok=True)
            output = memory_text(ram, width)
            path.write_text(output, encoding="ascii")
            report["outputs"][str(path)] = hashlib.sha256(output.encode("ascii")).hexdigest()
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
