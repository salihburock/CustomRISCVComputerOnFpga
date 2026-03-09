import struct

# The manually compiled Hex Code
instructions = [
    0x00f00093,  # 0x00: ADDI x1, x0, 15
    0x00a00113,  # 0x04: ADDI x2, x0, 10
    0x002081b3,  # 0x08: ADD  x3, x1, x2
    0x40208233,  # 0x0C: SUB  x4, x1, x2
    0x123452b7,  # 0x10: LUI  x5, 0x12345
    0x0c800393,  # 0x14: ADDI x7, x0, 200
    0x0033a023,  # 0x18: SW   x3, 0(x7)
    0x0003a403,  # 0x1C: LW   x8, 0(x7)
    0x00108463,  # 0x20: BEQ  x1, x1, 8
    0x06300493,  # 0x24: ADDI x9, x0, 99
    0x00c0056f,  # 0x28: JAL  x10, 12
    0x04d00593,  # 0x2C: ADDI x11, x0, 77
    0x00c0006f,  # 0x30: JAL  x0, 12
    0x02a00613,  # 0x34: ADDI x12, x0, 42
    0x00050067,  # 0x38: JALR x0, 0(x10)
    0x00000000  # 0x3C: HALT (Triggers UART Dump)
]

filename = "test_rom.bin"

print(f"Compiling {len(instructions)} instructions...")

with open(filename, "wb") as f:
    for i, instr in enumerate(instructions):
        # RISC-V uses Little-Endian byte order.
        # struct.pack("<I") converts the 32-bit hex into 4 little-endian bytes.
        byte_data = struct.pack("<I", instr)
        f.write(byte_data)

        # Print what is happening for debugging
        hex_str = "".join([f"{b:02X} " for b in byte_data])
        print(f"Address 0x{i * 4:02X}: {hex_str}  (Raw: 0x{instr:08X})")

print(f"\nSuccess! Wrote {len(instructions) * 4} bytes to {filename}")