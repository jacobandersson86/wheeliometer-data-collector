#pragma once
#include <stddef.h>
#include <stdbool.h>

// File system initialization
// Returns true on success, false on failure
bool fs_init();

// Unmount and cleanup file system
void fs_deinit();

// Check if file system is mounted
bool fs_is_mounted();

// Write data to a file (creates or overwrites)
// path: file path (e.g., "/data.txt")
// data: pointer to data to write
// size: size of data in bytes
// Returns: number of bytes written, or -1 on error
int fs_write_file(const char* path, const void* data, size_t size);

// Append data to a file
// path: file path
// data: pointer to data to append
// size: size of data in bytes
// Returns: number of bytes written, or -1 on error
int fs_append_file(const char* path, const void* data, size_t size);

// Read entire file into buffer
// path: file path
// buffer: output buffer
// buffer_size: size of output buffer
// Returns: number of bytes read, or -1 on error
int fs_read_file(const char* path, void* buffer, size_t buffer_size);

// Delete a file
// path: file path
// Returns: true on success, false on failure
bool fs_delete_file(const char* path);

// Check if file exists
// path: file path
// Returns: true if file exists, false otherwise
bool fs_file_exists(const char* path);

// Get file size in bytes
// path: file path
// Returns: file size in bytes, or -1 on error
long fs_get_file_size(const char* path);

// Get file system information
// total_bytes: output for total space (can be NULL)
// used_bytes: output for used space (can be NULL)
// Returns: true on success, false on failure
bool fs_get_info(size_t* total_bytes, size_t* used_bytes);

// List all files in the root directory
// Prints to console (useful for debugging)
void fs_list_files();

// Get list of all files in filesystem
// callback: function called for each file with filename
// user_data: pointer passed to callback
typedef void (*fs_file_callback_t)(const char* filename, void* user_data);
void fs_foreach_file(fs_file_callback_t callback, void* user_data);

// Format the file system (CAREFUL: deletes all data!)
// Returns: true on success, false on failure
bool fs_format();
