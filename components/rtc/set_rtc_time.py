#!/usr/bin/env python3
"""
RTC Time Setter for ESP32 with BM8563
Sets the current time on the RTC via serial connection.
Usage: python set_rtc_time.py [serial_port] [baud_rate]
Example: python set_rtc_time.py /dev/ttyUSB0 115200
"""

import serial
import sys
import time
from datetime import datetime

def find_serial_port():
    """Try to find a likely ESP32 serial port"""
    import glob
    # Common ESP32 serial port patterns
    patterns = [
        '/dev/ttyUSB*',
        '/dev/ttyACM*',
        '/dev/cu.usbserial*',
        '/dev/cu.SLAB_USBtoUART',
        'COM[0-9]*'
    ]

    for pattern in patterns:
        ports = glob.glob(pattern)
        if ports:
            return ports[0]
    return None

def set_rtc_time(port, baudrate=115200):
    """Send current time to ESP32 to set RTC"""
    try:
        ser = serial.Serial(port, baudrate, timeout=2)
        print(f"Connected to {port} at {baudrate} baud")

        # Wait a moment for connection to stabilize
        time.sleep(1)

        # Flush any existing data
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # Get current time
        now = datetime.now()

        # Format: SETRTC:YYYY,MM,DD,HH,MM,SS,WEEKDAY\n
        # weekday: 0=Sunday, 1=Monday, etc.
        weekday = now.weekday()
        if weekday == 6:  # Python: Monday=0, Sunday=6
            weekday = 0   # RTC: Sunday=0
        else:
            weekday += 1  # RTC: Monday=1, etc.

        command = f"SETRTC:{now.year},{now.month},{now.day},{now.hour},{now.minute},{now.second},{weekday}\n"

        print(f"Setting RTC to: {now.strftime('%Y-%m-%d %H:%M:%S')}")

        # Send command
        ser.write(command.encode())
        ser.flush()

        # Wait for response and read all available data
        time.sleep(1.0)

        # Read response
        response = ""
        timeout_count = 0
        while timeout_count < 10:  # Try for up to 1 second
            if ser.in_waiting > 0:
                response += ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                timeout_count = 0  # Reset timeout if we got data
            else:
                time.sleep(0.1)
                timeout_count += 1

        if response:
            print(f"Response from device:\n{response}")
        else:
            print("No response received from device.")
            print("Make sure RTC_USE_COMPILE_TIME is set to 0 and firmware is reflashed.")

        ser.close()

        if "OK" in response or "RTC set" in response:
            print("\n✓ RTC time set successfully!")
            return True
        else:
            print("\n⚠ Command sent but no confirmation received.")
            print("Check that the device is listening for serial commands.")
            return False

    except serial.SerialException as e:
        print(f"Serial error: {e}")
        return False
    except Exception as e:
        print(f"Error: {e}")
        return False

def main():
    # Get serial port from command line or auto-detect
    port = None
    baudrate = 115200

    if len(sys.argv) >= 2:
        port = sys.argv[1]
    if len(sys.argv) >= 3:
        baudrate = int(sys.argv[2])

    if not port:
        print("No serial port specified, attempting auto-detection...")
        port = find_serial_port()
        if not port:
            print("Could not auto-detect serial port.")
            print("Usage: python set_rtc_time.py [serial_port] [baud_rate]")
            print("Example: python set_rtc_time.py /dev/ttyUSB0 115200")
            sys.exit(1)
        print(f"Auto-detected port: {port}")

    success = set_rtc_time(port, baudrate)
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()
