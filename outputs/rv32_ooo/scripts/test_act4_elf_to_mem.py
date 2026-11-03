#!/usr/bin/env python3
"""Loader regression; synthetic ELF fixtures do not constitute ACT4 tests."""

import struct
import unittest
from act4_elf_to_mem import ElfError, RAM_BYTES, load_elf, memory_text


def executable(segments=None, **overrides):
    segments = segments or [(0, b"\x13\x00\x00\x00", 8, 5)]
    ident = bytearray(b"\x7fELF" + bytes((1, 1, 1)) + bytes(9))
    fields = dict(kind=2, machine=243, version=1, entry=0, phoff=52, shoff=0,
                  flags=0, ehsize=52, phsize=32, phnum=len(segments), shsize=0,
                  shnum=0, shstr=0)
    fields.update(overrides)
    header = struct.pack("<HHIIIIIHHHHHH", *fields.values())
    offset = 52 + len(segments) * 32
    phdrs = bytearray()
    payload = bytearray()
    for address, contents, memsz, permissions in segments:
        phdrs.extend(struct.pack("<IIIIIIII", 1, offset, address, address,
                                 len(contents), memsz, permissions, 1))
        payload.extend(contents)
        offset += len(contents)
    return bytes(ident) + header + phdrs + payload


class LoaderTests(unittest.TestCase):
    def reject(self, blob, reason):
        with self.assertRaisesRegex(ElfError, reason):
            load_elf(blob)

    def test_code_bss_and_word_row_byte_order(self):
        blob = executable([(0, bytes.fromhex("1300100093002000"), 12, 5),
                           (0x100, b"\x11\x22", 8, 6)])
        ram, report = load_elf(blob)
        self.assertEqual(len(ram), RAM_BYTES)
        self.assertEqual(ram[:12], bytes.fromhex("130010009300200000000000"))
        self.assertEqual(ram[0x100:0x108], b"\x11\x22" + bytes(6))
        self.assertEqual(report["segments"][1]["zero_fill_bytes"], 6)
        self.assertEqual(memory_text(ram, 4).splitlines()[:2], ["00100013", "00200093"])
        self.assertEqual(memory_text(ram, 8).splitlines()[0], "0020009300100013")
        self.assertEqual(len(memory_text(ram, 4).splitlines()), 16384)
        self.assertEqual(len(memory_text(ram, 8).splitlines()), 8192)

    def test_ram_limit_is_not_silently_truncated(self):
        self.reject(executable([(0, b"\x13\0\0\0", 4, 5),
                                (65534, b"abcd", 4, 6)]), "outside")

    def test_truncation(self):
        self.reject(b"\x7fELF", "truncated")
        self.reject(executable()[:60], "program-header")
        self.reject(executable()[:-1], "file extent")

    def test_target_and_entry(self):
        for field, value, reason in (("machine", 3, "RISC-V"), ("kind", 1, "ET_EXEC"),
                                     ("entry", 4, "reset PC"), ("flags", 1, "RVC")):
            with self.subTest(field=field):
                self.reject(executable(**{field: value}), reason)

    def test_wrong_class_endian_or_version(self):
        for byte, value in ((4, 2), (5, 2), (6, 0)):
            blob = bytearray(executable())
            blob[byte] = value
            self.reject(blob, "ELF32")

    def test_no_executable_mapping_at_reset(self):
        self.reject(executable([(0, b"abcd", 4, 6)]), "executable PT_LOAD")
        self.reject(executable([(4, b"abcd", 4, 5)]), "executable PT_LOAD")

    def test_overlap_and_filesize_errors(self):
        self.reject(executable([(0, b"abcd", 8, 5), (4, b"1234", 4, 6)]), "overlapping")
        self.reject(executable([(0, b"abcd", 2, 5)]), "file extent")

    def test_no_virtual_remapping(self):
        blob = bytearray(executable())
        struct.pack_into("<I", blob, 52 + 12, 16)
        self.reject(blob, "virtual and physical")

    def test_program_header_alignment(self):
        blob = bytearray(executable())
        struct.pack_into("<I", blob, 52 + 28, 3)
        self.reject(blob, "alignment")

    def test_last_ram_word_is_preserved(self):
        ram, _ = load_elf(executable([(0, b"\x13\0\0\0", 4, 5),
                                     (65532, b"\x11\x22\x33\x44", 4, 6)]))
        self.assertEqual(memory_text(ram, 4).splitlines()[-1], "44332211")


if __name__ == "__main__":
    unittest.main(verbosity=2)
