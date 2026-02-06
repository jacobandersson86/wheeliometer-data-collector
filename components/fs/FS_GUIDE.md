# File System Module

Simple interface for storing data on M5StickC Plus internal flash using SPIFFS (960KB partition).

## Quick Start

```cpp
#include <fs.hpp>

// Initialize
fs_init();

// Write file
const char* data = "Hello!";
fs_write_file("/test.txt", data, strlen(data));

// Read file
char buffer[64];
int bytes = fs_read_file("/test.txt", buffer, sizeof(buffer));

// Check space
size_t total, used;
fs_get_info(&total, &used);
```

## API Reference

### Initialization
- `bool fs_init()` - Initialize filesystem (call at startup)
- `bool fs_is_mounted()` - Check if mounted

### File Operations
- `int fs_write_file(path, data, size)` - Create/overwrite file
- `int fs_append_file(path, data, size)` - Append to file
- `int fs_read_file(path, buffer, size)` - Read entire file
- `bool fs_delete_file(path)` - Delete file
- `bool fs_file_exists(path)` - Check if file exists
- `long fs_get_file_size(path)` - Get file size in bytes

### System
- `bool fs_get_info(&total, &used)` - Get storage info
- `void fs_list_files()` - Print files to console
- `void fs_foreach_file(callback, data)` - Iterate through all files
- `bool fs_format()` - Format filesystem (DELETES ALL DATA)

## Binary Data Logging

```cpp
typedef struct {
    uint32_t timestamp;
    float accel_x, accel_y, accel_z;
} sensor_data_t;

// Log data
sensor_data_t sample = {millis(), 1.0, 2.0, 3.0};
fs_append_file("/log.bin", &sample, sizeof(sample));

// Read log
long size = fs_get_file_size("/log.bin");
int count = size / sizeof(sensor_data_t);
sensor_data_t* samples = malloc(size);
fs_read_file("/log.bin", samples, size);
```

## Notes

- File paths must start with `/` (e.g., `/data.txt`)
- Returns bytes written/read, or -1 on error
- Filesystem auto-formats on first use
- Max 5 files open simultaneously
- Built-in wear leveling
