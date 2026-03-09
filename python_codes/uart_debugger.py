import serial
import time
from collections import deque


SERIAL_PORT = 'COM11'  # Make sure this matches your Gowin dock
BAUD_RATE = 9600
HISTORY_SIZE = 15  # How many past values to keep

output_history = deque(maxlen=HISTORY_SIZE)


def print_debug_info(pc, instr, reg_a, reg_b, reg_out):
    opcode = (instr >> 4) & 0x0F
    operand = instr & 0x0F

    print("=" * 60)
    print(f"      SAP-1 BINARY DEBUG MONITOR")
    print("=" * 60)

    print(f" PC       : {pc:08b}  (Addr: {pc})")
    print(f" INSTR    : {instr:08b}  (Op: {opcode:04b} | Arg: {operand:04b})")
    print(f" REG A    : {reg_a:08b}  (Dec: {reg_a})")
    print(f" REG B    : {reg_b:08b}  (Dec: {reg_b})")
    print(f" OUTPUT   : {reg_out:08b}  (Dec: {reg_out})")
    print("-" * 60)

    # Visual Flags
    zero_flag = "ON" if reg_a == 0 else "OFF"
    print(f" FLAGS    : [ ZERO: {zero_flag} ]")
    print("-" * 60)

    history_list = list(output_history)
    print(f" OUT HISTORY ({len(history_list)}/{HISTORY_SIZE}):")
    print(f" {history_list}")
    print("=" * 60)
    print("\n")


try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
    print("Waiting for data...")

    while True:
        # 1. Sync on Header
        byte = ser.read(1)
        if byte == b'\xAA':
            # 2. Read Packet
            packet = ser.read(5)

            if len(packet) == 5:
                pc = packet[0]
                instr = packet[1]
                reg_a = packet[2]
                reg_b = packet[3]
                reg_out = packet[4]

                if len(output_history) == 0:
                    output_history.append(reg_out)
                elif output_history[-1] != reg_out:
                    output_history.append(reg_out)

                print_debug_info(pc, instr, reg_a, reg_b, reg_out)

except serial.SerialException as e:
    print(f"Error: {e}")
except KeyboardInterrupt:
    print("\nExiting...")