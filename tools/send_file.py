#!/usr/bin/env python3
"""
Send file to PicoCalc via serial connection

This script sends a file to the PicoCalc using a custom protocol
compatible with the 'recv' command on the device.

Protocol:
  1. CMD_START (0xAA)
  2. Filename length (1 byte)
  3. Filename (N bytes)
  4. File size (4 bytes, little-endian)
  5. For each chunk:
     - CMD_DATA (0xBB)
     - Chunk size (2 bytes, little-endian)
     - Data (N bytes)
     - Checksum (1 byte, XOR of all data bytes)
     - Wait for ACK (0xDD) or NAK (0xEE)
  6. CMD_END (0xCC)
  7. Wait for final ACK

Usage:
  python3 send_file.py /dev/ttyACM0 filename.txt
  python3 send_file.py COM3 image.raw
"""

import sys
import time
import serial
import os

# Protocol commands
CMD_START = 0xAA
CMD_DATA = 0xBB
CMD_END = 0xCC
CMD_ACK = 0xDD
CMD_NAK = 0xEE

# Transfer settings
CHUNK_SIZE = 512
MAX_RETRIES = 3

def calculate_checksum(data):
    """Calculate XOR checksum of data"""
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum

def send_file(port, filename, baudrate=115200):
    """Send file to PicoCalc"""

    if not os.path.exists(filename):
        print(f"Error: File '{filename}' not found.")
        return False

    file_size = os.path.getsize(filename)
    basename = os.path.basename(filename)

    print(f"Sending file: {basename}")
    print(f"Size: {file_size} bytes")
    print(f"Port: {port}")
    print(f"Baudrate: {baudrate}")
    print()

    try:
        # Open serial port
        print(f"Opening serial port {port}...")
        ser = serial.Serial(port, baudrate, timeout=5)
        time.sleep(1.0)  # Wait for port to stabilize

        # Flush any existing data in buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print("Serial port opened and buffers flushed.")
        print()
        print("IMPORTANT: You have 5 seconds to run 'recv' command on PicoCalc NOW!")
        for i in range(5, 0, -1):
            print(f"Starting in {i}...")
            time.sleep(1)
        print()

        # Send START command
        print(f"Sending START command (0x{CMD_START:02X})...")
        ser.write(bytes([CMD_START]))
        ser.flush()

        # Send filename length
        filename_bytes = basename.encode('utf-8')
        print(f"Sending filename length: {len(filename_bytes)}")
        ser.write(bytes([len(filename_bytes)]))
        ser.flush()

        # Send filename
        print(f"Sending filename: {basename}")
        ser.write(filename_bytes)
        ser.flush()

        # Send file size (4 bytes, little-endian)
        size_bytes = file_size.to_bytes(4, byteorder='little')
        print(f"Sending file size: {file_size} bytes")
        ser.write(size_bytes)
        ser.flush()

        # Wait for ACK
        print("Waiting for ACK after header...")
        response = ser.read(1)
        if len(response) == 0:
            print("Error: No ACK received after header (timeout)")
            print("Device may not be in receive mode or wrong port")
            return False

        print(f"Received byte: 0x{response[0]:02X}")

        # If we got unexpected data, read more to see what's coming
        if response[0] != CMD_ACK:
            print(f"Error: Expected ACK (0x{CMD_ACK:02X}), got 0x{response[0]:02X}")
            # Read any additional bytes to see what the device is sending
            time.sleep(0.1)
            extra = ser.read(ser.in_waiting)
            if len(extra) > 0:
                print(f"Additional bytes in buffer: {' '.join(f'0x{b:02X}' for b in extra)}")
                print(f"As ASCII: {extra}")
            return False

        print(f"Header acknowledged. Starting data transfer...")
        print()

        # Send file data in chunks
        bytes_sent = 0
        with open(filename, 'rb') as f:
            while bytes_sent < file_size:
                # Read chunk
                chunk = f.read(CHUNK_SIZE)
                chunk_size = len(chunk)

                # Try sending chunk with retries
                retry_count = 0
                while retry_count < MAX_RETRIES:
                    # Send DATA command
                    ser.write(bytes([CMD_DATA]))

                    # Send chunk size (2 bytes, little-endian)
                    size_bytes = chunk_size.to_bytes(2, byteorder='little')
                    ser.write(size_bytes)

                    # Send data
                    ser.write(chunk)

                    # Send checksum
                    checksum = calculate_checksum(chunk)
                    ser.write(bytes([checksum]))

                    # Wait for ACK/NAK
                    response = ser.read(1)
                    if len(response) == 0:
                        print(f"\nTimeout waiting for response (chunk at {bytes_sent})")
                        retry_count += 1
                        continue

                    if response[0] == CMD_ACK:
                        bytes_sent += chunk_size
                        break
                    elif response[0] == CMD_NAK:
                        print(f"\nNAK received, retrying chunk at {bytes_sent}...")
                        retry_count += 1
                        continue
                    else:
                        print(f"\nUnexpected response: 0x{response[0]:02X}")
                        retry_count += 1
                        continue

                if retry_count >= MAX_RETRIES:
                    print(f"\nError: Max retries exceeded at byte {bytes_sent}")
                    return False

                # Show progress
                progress = (bytes_sent * 100) // file_size
                print(f"\rProgress: {bytes_sent}/{file_size} bytes ({progress}%)", end='', flush=True)

        print()  # New line after progress

        # Send END command
        ser.write(bytes([CMD_END]))

        # Wait for final ACK
        response = ser.read(1)
        if len(response) == 0 or response[0] != CMD_ACK:
            print("Warning: No final ACK received")

        print()
        print("Transfer completed successfully!")

        ser.close()
        return True

    except serial.SerialException as e:
        print(f"Serial error: {e}")
        return False
    except Exception as e:
        print(f"Error: {e}")
        return False

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 send_file.py <serial_port> <filename>")
        print()
        print("Examples:")
        print("  python3 send_file.py /dev/ttyACM0 readme.txt")
        print("  python3 send_file.py COM3 image.raw")
        print()
        sys.exit(1)

    port = sys.argv[1]
    filename = sys.argv[2]

    # Check if pyserial is installed
    try:
        import serial
    except ImportError:
        print("Error: pyserial is not installed.")
        print("Install with: pip install pyserial")
        sys.exit(1)

    success = send_file(port, filename)
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()
