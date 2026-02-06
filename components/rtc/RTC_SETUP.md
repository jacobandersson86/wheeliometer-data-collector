# RTC (BM8563) Time Setup Guide

The M5StickC Plus has a BM8563 RTC chip that can maintain time and date. This guide explains how to set the time for the first time.

## Two Methods Available

### Method 1: Serial Command (Default, Most Accurate)

This method uses a Python script to set the exact current time via serial after flashing.

**Pros:**
- Accurate to the second
- Can update time without reflashing
- Can be run anytime

**Cons:**
- Requires Python 3 with pyserial
- Extra step after flashing

**How to use:**

1. Flash your firmware (serial mode is the default)
2. Install pyserial if not already installed:
   ```bash
   pip3 install pyserial
   ```
3. Run the Python script:
   ```bash
   # Auto-detect serial port
   python3 components/rtc/set_rtc_time.py

   # Or specify port and baud rate
   python3 components/rtc/set_rtc_time.py /dev/ttyUSB0 115200
   ```

The script will:
- Connect to your M5StickC Plus via serial
- Send the current system time
- Set the RTC accordingly

**Serial Command Format:**
If you want to set the time manually via serial terminal:
```
SETRTC:YYYY,MM,DD,HH,MM,SS,WEEKDAY
```
Example: `SETRTC:2026,2,6,14,30,0,4` (Thursday, Feb 6, 2026 at 14:30:00)

Weekday values: 0=Sunday, 1=Monday, 2=Tuesday, 3=Wednesday, 4=Thursday, 5=Friday, 6=Saturday

### Method 2: Compile-Time Timestamp (Simpler)

This method automatically sets the RTC to the compile time when you flash the firmware.

**Pros:**
- No extra steps needed
- Works offline
- Automatic on every flash

**Cons:**
- Time will be slightly off (compile time vs flash time)
- Less accurate (typically 1-2 minutes off)

**How to use:**
1. In `main/main.cpp`, change `rtc_init(RTC_MODE_SERIAL)` to `rtc_init(RTC_MODE_COMPILE_TIME)`
2. Flash your firmware
3. The RTC will be set to the compile timestamp

**Accuracy:** The time will be set to when the code was compiled, which is typically 1-2 minutes before flashing. Good enough for most applications where exact time isn't critical.

## RTC Behavior

- **First boot:** If RTC has invalid time (year < 2020 or > 2100), it will be initialized using your chosen method
- **Subsequent boots:** RTC maintains time even after power cycles (if backup power available)
- **Time persistence:** The BM8563 can maintain time with backup battery or capacitor

## Checking RTC Time

The current RTC time is displayed on the terminal at startup. You can also use the RTC module functions:

```cpp
// Get formatted time string
char time_buffer[32];
rtc_get_time_string(time_buffer, sizeof(time_buffer));
terminal_write(time_buffer);

// Or access directly via M5
auto dt = M5.Rtc.getDateTime();
printf("Current time: %04d-%02d-%02d %02d:%02d:%02d\n",
       dt.date.year, dt.date.month, dt.date.date,
       dt.time.hours, dt.time.minutes, dt.time.seconds);
```

## Example Use Cases

**For development/testing:** Use Method 2 (compile-time). Quick and easy, just change the mode in code.

**For deployment (default):** Use Method 1 (serial command) for initial setup, then the RTC will maintain accurate time.

## Troubleshooting

**"RTC not detected!"**
- Check I2C connections
- Verify BM8563 is properly powered
- Check M5Unified configuration enables internal_rtcM5StickC Plus

**Time keeps resetting:**
- RTC may not have backup power
- Check for backup battery or supercapacitor on your devkit
- Consider adding NTP sync for periodic time updates

**Python script can't find port:**
- Specify the port explicitly: `python set_rtc_time.py /dev/ttyUSB0`
- On macOS: Check `/dev/cu.usbserial*` or `/dev/cu.SLAB_USBtoUART`
- On Linux: Check `/dev/ttyUSB*` or `/dev/ttyACM*`
- On Windows: Check Device Manager for COM port number
