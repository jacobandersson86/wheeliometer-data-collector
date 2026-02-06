#include "fs.hpp"
#include "terminal.hpp"
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <dirent.h>
#include "esp_spiffs.h"
#include "esp_log.h"

static const char* TAG = "fs";
static bool fs_mounted = false;
static const char* base_path = "/spiffs";

bool fs_init() {
    if (fs_mounted) {
        ESP_LOGW(TAG, "File system already mounted");
        return true;
    }

    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
        .base_path = base_path,
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return false;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "SPIFFS mounted successfully");
    ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);

    terminal_write("File system initialized: %d/%d bytes used", used, total);

    fs_mounted = true;
    return true;
}

void fs_deinit() {
    if (!fs_mounted) {
        return;
    }

    esp_vfs_spiffs_unregister(NULL);
    fs_mounted = false;
    ESP_LOGI(TAG, "SPIFFS unmounted");
}

bool fs_is_mounted() {
    return fs_mounted;
}

int fs_write_file(const char* path, const void* data, size_t size) {
    if (!fs_mounted) {
        ESP_LOGE(TAG, "File system not mounted");
        return -1;
    }

    char full_path[128];
    snprintf(full_path, sizeof(full_path), "%s%s", base_path, path);

    FILE* f = fopen(full_path, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", full_path);
        return -1;
    }

    size_t written = fwrite(data, 1, size, f);
    fclose(f);

    if (written != size) {
        ESP_LOGE(TAG, "Write error: expected %d bytes, wrote %d", size, written);
        return -1;
    }

    ESP_LOGI(TAG, "Wrote %d bytes to %s", written, path);
    return written;
}

int fs_append_file(const char* path, const void* data, size_t size) {
    if (!fs_mounted) {
        ESP_LOGE(TAG, "File system not mounted");
        return -1;
    }

    char full_path[128];
    snprintf(full_path, sizeof(full_path), "%s%s", base_path, path);

    FILE* f = fopen(full_path, "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for appending: %s", full_path);
        return -1;
    }

    size_t written = fwrite(data, 1, size, f);
    fclose(f);

    if (written != size) {
        ESP_LOGE(TAG, "Append error: expected %d bytes, wrote %d", size, written);
        return -1;
    }

    ESP_LOGD(TAG, "Appended %d bytes to %s", written, path);
    return written;
}

int fs_read_file(const char* path, void* buffer, size_t buffer_size) {
    if (!fs_mounted) {
        ESP_LOGE(TAG, "File system not mounted");
        return -1;
    }

    char full_path[128];
    snprintf(full_path, sizeof(full_path), "%s%s", base_path, path);

    FILE* f = fopen(full_path, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading: %s", full_path);
        return -1;
    }

    size_t read = fread(buffer, 1, buffer_size, f);
    fclose(f);

    ESP_LOGI(TAG, "Read %d bytes from %s", read, path);
    return read;
}

bool fs_delete_file(const char* path) {
    if (!fs_mounted) {
        ESP_LOGE(TAG, "File system not mounted");
        return false;
    }

    char full_path[128];
    snprintf(full_path, sizeof(full_path), "%s%s", base_path, path);

    if (unlink(full_path) == 0) {
        ESP_LOGI(TAG, "Deleted file: %s", path);
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to delete file: %s", path);
        return false;
    }
}

bool fs_file_exists(const char* path) {
    if (!fs_mounted) {
        return false;
    }

    char full_path[128];
    snprintf(full_path, sizeof(full_path), "%s%s", base_path, path);

    struct stat st;
    return (stat(full_path, &st) == 0);
}

long fs_get_file_size(const char* path) {
    if (!fs_mounted) {
        return -1;
    }

    char full_path[128];
    snprintf(full_path, sizeof(full_path), "%s%s", base_path, path);

    struct stat st;
    if (stat(full_path, &st) == 0) {
        return st.st_size;
    }
    return -1;
}

bool fs_get_info(size_t* total_bytes, size_t* used_bytes) {
    if (!fs_mounted) {
        return false;
    }

    size_t total = 0, used = 0;
    esp_err_t ret = esp_spiffs_info(NULL, &total, &used);

    if (ret != ESP_OK) {
        return false;
    }

    if (total_bytes) *total_bytes = total;
    if (used_bytes) *used_bytes = used;

    return true;
}

void fs_foreach_file(fs_file_callback_t callback, void* user_data) {
    if (!fs_mounted || !callback) {
        return;
    }

    DIR* dir = opendir(base_path);
    if (!dir) {
        return;
    }

    struct dirent* entry;
    while ((entry = readdir(dir)) != NULL) {
        callback(entry->d_name, user_data);
    }

    closedir(dir);
}

void fs_list_files() {
    if (!fs_mounted) {
        printf("File system not mounted\n");
        return;
    }

    DIR* dir = opendir(base_path);
    if (!dir) {
        printf("Failed to open directory\n");
        return;
    }

    printf("\nFiles in %s:\n", base_path);
    printf("------------------------\n");

    struct dirent* entry;
    int count = 0;
    while ((entry = readdir(dir)) != NULL) {
        char full_path[300];
        snprintf(full_path, sizeof(full_path), "%s/%s", base_path, entry->d_name);

        struct stat st;
        if (stat(full_path, &st) == 0) {
            printf("  %s (%ld bytes)\n", entry->d_name, st.st_size);
            count++;
        }
    }

    closedir(dir);
    printf("------------------------\n");
    printf("Total files: %d\n\n", count);
}

bool fs_format() {
    ESP_LOGW(TAG, "Formatting file system...");

    if (fs_mounted) {
        fs_deinit();
    }

    esp_err_t ret = esp_spiffs_format(NULL);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Format failed: %s", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "Format successful");

    // Remount after format
    return fs_init();
}
