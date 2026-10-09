"""Validate image corruption failures without requiring an ARM compiler."""

import contextlib
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import verify_artifacts as artifacts


def record(kind, payload=b"", offset=0):
    data = bytes([len(payload)]) + offset.to_bytes(2, "big") + bytes([kind]) + payload
    return ":" + (data + bytes([-sum(data) & 255])).hex().upper()


def image_text(binary):
    return "\n".join([record(4, bytes.fromhex("0800")), record(0, binary), record(1)])


class ArtifactTests(unittest.TestCase):
    def setUp(self):
        self.binary = (
            struct.pack("<II", artifacts.RAM_BASE + 4096, artifacts.FLASH_BASE + 9)
            + artifacts.BUILD_TAG
        )

    def test_valid_image_and_vectors(self):
        self.assertEqual(artifacts.decode_hex(image_text(self.binary)), self.binary)
        self.assertEqual(
            artifacts.validate_binary(self.binary),
            (artifacts.RAM_BASE + 4096, artifacts.FLASH_BASE + 9),
        )

    def test_invalid_records_are_rejected(self):
        valid = image_text(self.binary)
        cases = [
            valid[:-2] + "00",
            valid.rsplit("\n", 1)[0],
            valid + "\n" + record(0, b"x"),
            valid.replace(":020000040800F2", record(4, bytes.fromhex("0801"))),
            valid.replace(":020000040800F2", record(4, b"\x08")),
            "invalid",
            ":",
            record(6) + "\n" + record(1),
        ]
        for text in cases:
            with self.subTest(text=text), self.assertRaises(ValueError):
                artifacts.decode_hex(text)

    def test_conflicting_records_and_sparse_bytes(self):
        prefix = record(4, bytes.fromhex("0800"))
        with self.assertRaisesRegex(ValueError, "Conflicting"):
            artifacts.decode_hex(
                "\n".join([prefix, record(0, b"a"), record(0, b"b"), record(1)])
            )
        self.assertEqual(
            artifacts.decode_hex(
                "\n".join([prefix, record(0, b"a"), record(0, b"b", 2), record(1)])
            ),
            b"a\xffb",
        )

    def test_invalid_vectors_size_and_build_tag(self):
        for binary in [
            b"",
            self.binary[:7],
            b"\x00" * len(self.binary),
            struct.pack("<II", artifacts.RAM_BASE, artifacts.FLASH_BASE + 9)
            + artifacts.BUILD_TAG,
            struct.pack("<II", artifacts.RAM_BASE + 4096, artifacts.FLASH_BASE + 8)
            + artifacts.BUILD_TAG,
            struct.pack("<II", artifacts.RAM_BASE + 4096, artifacts.FLASH_BASE + 999)
            + artifacts.BUILD_TAG,
        ]:
            with self.subTest(binary=binary), self.assertRaises(ValueError):
                artifacts.validate_binary(binary)

    def test_saved_report_must_match_and_cli_is_read_only(self):
        temporary_root = Path(__file__).resolve().parents[1] / "build_host"
        temporary_root.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=temporary_root) as directory:
            folder = Path(directory)
            hex_path, bin_path, saved = (
                folder / "test.hex",
                folder / "test.bin",
                folder / "saved.json",
            )
            hex_path.write_text(image_text(self.binary), encoding="ascii")
            bin_path.write_bytes(self.binary)
            saved.write_text(
                json.dumps(
                    {
                        "hex_sha256": artifacts.sha256(hex_path),
                        "bin_sha256": artifacts.sha256(bin_path),
                        "ram_link_bytes": 4096,
                    }
                ),
                encoding="utf-8",
            )
            before = {path.name: path.read_bytes() for path in folder.iterdir()}
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(
                    artifacts.main(
                        [
                            "--hex",
                            str(hex_path),
                            "--bin",
                            str(bin_path),
                            "--saved-report",
                            str(saved),
                        ]
                    ),
                    0,
                )
            self.assertEqual(
                before, {path.name: path.read_bytes() for path in folder.iterdir()}
            )
            bin_path.write_bytes(self.binary + b"x")
            with self.assertRaisesRegex(ValueError, "images differ"):
                artifacts.verify_images(hex_path, bin_path, saved_report=saved)
            bin_path.write_bytes(self.binary)
            saved.write_text("{}", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "checksums"):
                artifacts.verify_images(hex_path, bin_path, saved_report=saved)

    def test_elf_alignment_gaps_mismatch_and_ram_limit(self):
        class Section(dict):
            def __init__(self, address, payload):
                super().__init__(sh_addr=address, sh_flags=2, sh_type="SHT_PROGBITS")
                self.payload = payload

            def data(self):
                return self.payload

        class Segment(dict):
            def section_in_segment(self, section):
                return True

        class Symbol(dict):
            def __init__(self, name, value):
                super().__init__(st_value=value)
                self.name = name

        symbols = [
            Symbol("_image_ram_start", artifacts.RAM_BASE),
            Symbol("_image_ram_end", artifacts.RAM_BASE + 4096),
        ]
        binary = self.binary[:8] + b"\xff" * 4 + self.binary[8:]
        elf = SimpleNamespace(
            get_section_by_name=lambda name: SimpleNamespace(
                iter_symbols=lambda: symbols
            ),
            iter_sections=lambda: [
                Section(artifacts.FLASH_BASE, binary[:8]),
                Section(artifacts.FLASH_BASE + 12, binary[12:]),
            ],
            iter_segments=lambda: [
                Segment(
                    p_type="PT_LOAD",
                    p_filesz=len(binary),
                    p_paddr=artifacts.FLASH_BASE,
                    p_vaddr=artifacts.FLASH_BASE,
                )
            ],
        )
        module = ModuleType("elftools.elf.elffile")
        module.ELFFile = lambda stream: elf
        with patch.dict(sys.modules, {"elftools.elf.elffile": module}):
            self.assertEqual(artifacts.linked_ram(__file__, binary), 4096)
            with self.assertRaisesRegex(ValueError, "does not match"):
                artifacts.linked_ram(__file__, binary[:-1] + b"x")
            symbols[1]["st_value"] = artifacts.RAM_BASE + artifacts.RAM_LIMIT + 1
            with self.assertRaisesRegex(ValueError, "exceeds RAM"):
                artifacts.linked_ram(__file__, binary)


if __name__ == "__main__":
    unittest.main()
