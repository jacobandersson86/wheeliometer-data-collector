# Wheeliometer Data Collector

Data collection firmware for the M5StickC Plus. This project collects accelerometer and gyroscope data from the built-in MEMS sensor for analysis and processing.

## Hardware Requirements

- **M5StickC Plus** - Compact ESP32 development board with built-in IMU sensor

## Getting Started

### Prerequisites

1. **ESP-IDF v5.5.2** - Install the ESP-IDF development framework:
   - Follow the [official ESP-IDF installation guide](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32/get-started/index.html)
   - Make sure to install version 5.5.2 specifically

2. **Clone the Repository**
   ```bash
   git clone --recursive https://github.com/jacobandersson86/wheeliometer-data-collector.git
   cd wheeliometer-project/data-collector
   ```

   **Note:** Use `--recursive` to clone all submodule dependencies (M5Unified, M5GFX, etc.)

### Building, Flashing, and Monitoring

For detailed instructions on building, flashing, and monitoring your project:
- [ESP-IDF Build System Documentation](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32/api-guides/build-system.html)
- [ESP-IDF Flashing Guide](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32/get-started/index.html#step-9-flash-onto-the-device)

**Quick commands:**
```bash
idf.py build        # Build the project
idf.py flash        # Flash to device
idf.py monitor      # Open serial monitor
idf.py flash monitor # Flash and monitor in one command
```

## Usage

### Setting Up RTC Time

The M5StickC Plus includes a BM8563 RTC chip for timekeeping. You should set the time when first using the device.

See [RTC Setup Guide](components/rtc/RTC_SETUP.md) for detailed instructions on setting the date and time.

### Using the File System

The device has 960KB of internal flash storage for saving collected sensor data. The file system is automatically initialized at startup.

See [File System Guide](components/fs/FS_GUIDE.md) for API reference and usage examples.

## Project Structure

```
data-collector/
├── components/          # Custom components
│   ├── buttons/        # Button handling
│   ├── fs/            # File system (see FS_GUIDE.md)
│   ├── rtc/           # Real-time clock (see RTC_SETUP.md)
│   └── terminal/      # Display terminal output
├── main/              # Main application code
├── managed_components/ # External dependencies
└── partitions.csv     # Flash partition table
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
