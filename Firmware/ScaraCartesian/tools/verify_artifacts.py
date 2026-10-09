"""Verify an explicit firmware image without touching hardware or release files."""

import argparse
import hashlib
import json
from pathlib import Path
import struct

ROOT = Path(__file__).resolve().parents[1]
IMAGE_NAME = "SCARA_Cartesian_NC_Home_v5_R9"
BUILD_TAG = b"SCARA_CART_NC_HOME_V5_R9"
FLASH_BASE = 0x08000000
FLASH_LIMIT = 65536
RAM_BASE = 0x20000000
RAM_LIMIT = 20480


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def decode_hex(text):
    """Read Intel HEX records, checking checksums, bounds and termination."""
    memory = {}
    base = 0
    eof = False
    for number, line in enumerate(text.splitlines(), 1):
        if not line:
            continue
        require(not eof, f"Record after EOF at line {number}")
        require(line.startswith(":"), f"Missing record prefix at line {number}")
        try:
            record = bytes.fromhex(line[1:])
        except ValueError as error:
            raise ValueError(f"Invalid HEX at line {number}") from error
        require(len(record) >= 5, f"Short record at line {number}")
        count = record[0]
        offset = int.from_bytes(record[1:3], "big")
        kind = record[3]
        require(len(record) == count + 5, f"Invalid record length at line {number}")
        require(sum(record) % 256 == 0, f"Checksum mismatch at line {number}")
        payload = record[4 : 4 + count]
        if kind == 0:
            for index, value in enumerate(payload):
                address = base + offset + index
                require(
                    FLASH_BASE <= address < FLASH_BASE + FLASH_LIMIT,
                    f"Data outside flash at line {number}",
                )
                require(
                    address not in memory or memory[address] == value,
                    f"Conflicting data at line {number}",
                )
                memory[address] = value
        elif kind in (2, 4):
            require(
                count == 2 and offset == 0, f"Invalid address record at line {number}"
            )
            base = int.from_bytes(payload, "big") << (4 if kind == 2 else 16)
        elif kind == 1:
            require(count == 0 and offset == 0, f"Invalid EOF at line {number}")
            eof = True
        elif kind in (3, 5):
            require(
                count == 4 and offset == 0, f"Invalid entry record at line {number}"
            )
        else:
            raise ValueError(f"Unsupported record type {kind} at line {number}")
    require(eof, "Missing EOF record")
    require(memory and min(memory) == FLASH_BASE, "Image must start at 0x08000000")
    return bytes(
        memory.get(address, 255) for address in range(FLASH_BASE, max(memory) + 1)
    )


def validate_binary(binary):
    require(8 <= len(binary) < FLASH_LIMIT, "Invalid flash image size")
    stack_pointer, reset = struct.unpack_from("<II", binary)
    require(
        RAM_BASE < stack_pointer <= RAM_BASE + RAM_LIMIT,
        "Initial stack pointer is outside RAM",
    )
    require(
        reset & 1 and FLASH_BASE <= (reset & ~1) < FLASH_BASE + len(binary),
        "Invalid Thumb reset vector",
    )
    require(BUILD_TAG in binary, "Firmware build tag is missing")
    return stack_pointer, reset


def linked_ram(elf_path, binary):
    """Accept RAM symbols only when this ELF contains the supplied image."""
    from elftools.elf.elffile import ELFFile

    with Path(elf_path).open("rb") as stream:
        elf = ELFFile(stream)
        symbol_table = elf.get_section_by_name(".symtab")
        require(symbol_table is not None, "ELF has no symbol table")
        symbols = {
            symbol.name: symbol["st_value"]
            for symbol in symbol_table.iter_symbols()
            if symbol.name in ("_image_ram_start", "_image_ram_end")
        }
        require(len(symbols) == 2, "ELF is missing RAM image symbols")
        image = {}
        sections = list(elf.iter_sections())
        for segment in elf.iter_segments():
            if segment["p_type"] != "PT_LOAD" or not segment["p_filesz"]:
                continue
            start = segment["p_paddr"]
            if not FLASH_BASE <= start < FLASH_BASE + FLASH_LIMIT:
                continue
            # objcopy emits allocated sections at their load addresses and fills
            # alignment holes with 0xff. PT_LOAD file padding can contain zeros.
            for section in sections:
                if not section["sh_flags"] & 2 or section["sh_type"] == "SHT_NOBITS":
                    continue
                if not segment.section_in_segment(section):
                    continue
                address = start + section["sh_addr"] - segment["p_vaddr"]
                payload = section.data()
                require(
                    FLASH_BASE <= address
                    and address + len(payload) <= FLASH_BASE + FLASH_LIMIT,
                    "ELF flash section exceeds flash",
                )
                for index, value in enumerate(payload):
                    position = address + index
                    require(
                        position not in image or image[position] == value,
                        "ELF flash sections overlap inconsistently",
                    )
                    image[position] = value
        require(image and min(image) == FLASH_BASE, "ELF has no firmware flash image")
        rebuilt = bytes(
            image.get(address, 255) for address in range(FLASH_BASE, max(image) + 1)
        )
        require(rebuilt == binary, "ELF flash image does not match BIN")
    start, end = symbols["_image_ram_start"], symbols["_image_ram_end"]
    require(
        start == RAM_BASE and start < end <= RAM_BASE + RAM_LIMIT,
        "Linked image exceeds RAM",
    )
    return end - start


def verify_images(hex_path, bin_path, elf_path=None, saved_report=None):
    hex_path, bin_path = Path(hex_path), Path(bin_path)
    binary = bin_path.read_bytes()
    require(
        decode_hex(hex_path.read_text(encoding="ascii")) == binary,
        "HEX and BIN images differ",
    )
    stack_pointer, reset = validate_binary(binary)
    hashes = {"hex_sha256": sha256(hex_path), "bin_sha256": sha256(bin_path)}
    if elf_path is not None:
        ram_bytes = linked_ram(elf_path, binary)
        ram_source = "linked ELF matching image bytes"
    else:
        require(saved_report is not None, "Supply --elf or a matching saved report")
        saved = json.loads(Path(saved_report).read_text(encoding="utf-8"))
        require(
            all(saved.get(key) == value for key, value in hashes.items()),
            "Saved report does not match image checksums",
        )
        ram_bytes = saved["ram_link_bytes"]
        require(
            isinstance(ram_bytes, int) and 0 < ram_bytes <= RAM_LIMIT,
            "Invalid saved RAM size",
        )
        ram_source = "saved linked report matching image checksums"
    report = {
        "build": BUILD_TAG.decode("ascii"),
        "flash_bytes": len(binary),
        "flash_limit": FLASH_LIMIT,
        "ram_link_bytes": ram_bytes,
        "ram_limit": RAM_LIMIT,
        "ram_check_source": ram_source,
        "base": hex(FLASH_BASE),
        "initial_sp": hex(stack_pointer),
        "reset_vector": hex(reset),
        **hashes,
        "validation": [
            "HEX checksums, record grammar and flash bounds",
            "HEX/BIN byte equality",
            "Stack and Thumb reset vectors",
            "Build tag",
            "RAM size tied to matching firmware image",
        ],
        "real_hardware_tested": False,
    }
    if elf_path is not None:
        report["elf_sha256"] = sha256(elf_path)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--hex", type=Path, default=ROOT / "Hex" / (IMAGE_NAME + ".hex")
    )
    parser.add_argument(
        "--bin", type=Path, default=ROOT / "Hex" / (IMAGE_NAME + ".bin")
    )
    parser.add_argument("--elf", type=Path, help="Linked ELF of this exact image")
    parser.add_argument(
        "--saved-report", type=Path, default=ROOT / "Hex" / "verification_nc_v5_R9.json"
    )
    parser.add_argument(
        "--output", type=Path, help="Write a report only when explicitly requested"
    )
    args = parser.parse_args(argv)
    try:
        report = verify_images(args.hex, args.bin, args.elf, args.saved_report)
    except (OSError, ValueError, KeyError, ImportError) as error:
        parser.exit(1, f"Firmware verification failed: {error}\n")
    serialized = json.dumps(report, indent=2) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(serialized, encoding="utf-8")
    print(serialized, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
