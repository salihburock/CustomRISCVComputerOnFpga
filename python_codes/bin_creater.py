# 4 Megabytes = 4,194,304 bytes
TARGET_SIZE_BYTES = 4 * 1024 * 1024

print("Generating 4MB hardware test pattern...")

with open("flash_test_4MB.bin", "wb") as f:
    for address in range(0, TARGET_SIZE_BYTES, 16):
        chunk = bytearray()

        # 1. Alignment Test: 4 bytes (0xDEADBEEF)
        chunk.extend(bytes.fromhex("DEADBEEF"))

        # 2. Clock Edge Test: 2 bytes (0xAA55)
        # 0xAA = 10101010, 0x55 = 01010101
        chunk.extend(bytes.fromhex("AA55"))

        # 3. Address Tracker: 4 bytes (The current memory address)
        chunk.extend(address.to_bytes(4, byteorder='big'))

        # 4. Human Readable Text & Padding: 6 bytes
        chunk.extend(b"TEST\r\n")

        # Write the 16-byte chunk to the file
        f.write(chunk)

print("flash_test_4MB.bin successfully generated!")