#include "webserver.hpp"
#include "terminal.hpp"
#include "fs.hpp"
#include "esp_http_server.h"
#include "esp_log.h"
#include <string.h>
#include <sys/stat.h>
#include <dirent.h>

static const char* TAG = "WEBSERVER";
static httpd_handle_t server = NULL;

// Simple HTML page
static const char* html_page =
    "<!DOCTYPE html>"
    "<html>"
    "<head>"
    "<meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
    "<title>Wheeliometer</title>"
    "<style>"
    "body { font-family: Arial, sans-serif; margin: 40px; background: #f0f0f0; }"
    "h1 { color: #333; }"
    ".container { background: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); max-width: 800px; }"
    ".file-list { margin-top: 20px; }"
    ".file-item { padding: 10px; margin: 5px 0; background: #f9f9f9; border-radius: 4px; display: flex; justify-content: space-between; align-items: center; }"
    ".file-name { flex-grow: 1; font-family: monospace; }"
    ".file-size { color: #666; margin: 0 10px; }"
    "button { padding: 5px 15px; margin: 0 5px; cursor: pointer; border: none; border-radius: 4px; }"
    ".btn-download { background: #4CAF50; color: white; }"
    ".btn-delete { background: #f44336; color: white; }"
    ".btn-refresh { background: #2196F3; color: white; padding: 10px 20px; margin-top: 10px; }"
    ".btn-delete-all { background: #ff9800; color: white; padding: 10px 20px; margin: 10px 5px; }"
    ".btn-format { background: #9c27b0; color: white; padding: 10px 20px; margin: 10px 5px; }"
    ".loading { color: #666; font-style: italic; }"
    ".actions { margin-top: 20px; padding-top: 20px; border-top: 2px solid #e0e0e0; }"
    "</style>"
    "</head>"
    "<body>"
    "<div class='container'>"
    "<h1>Wheeliometer Data Collector</h1>"
    "<p>Download or delete your data files</p>"
    "<button class='btn-refresh' onclick='loadFiles()'>Refresh File List</button>"
    "<div class='file-list' id='fileList'>"
    "<p class='loading'>Loading files...</p>"
    "</div>"
    "<div class='actions'>"
    "<button class='btn-delete-all' onclick='deleteAll()'>Delete All Files</button>"
    "<button class='btn-format' onclick='formatFS()'>Format Filesystem</button>"
    "</div>"
    "</div>"
    "<script>"
    "function loadFiles() {"
    "  document.getElementById('fileList').innerHTML = '<p class=\"loading\">Loading files...</p>';"
    "  fetch('/api/files')"
    "    .then(r => r.json())"
    "    .then(data => {"
    "      let html = '';"
    "      if (data.files.length === 0) {"
    "        html = '<p>No files found</p>';"
    "      } else {"
    "        data.files.forEach(f => {"
    "          html += '<div class=\"file-item\">';"
    "          html += '<span class=\"file-name\">' + f.name + '</span>';"
    "          html += '<span class=\"file-size\">' + f.size + ' bytes</span>';"
    "          html += '<button class=\"btn-download\" onclick=\"downloadFile(\\'' + f.name + '\\')\">Download</button>';"
    "          html += '<button class=\"btn-delete\" onclick=\"deleteFile(\\'' + f.name + '\\')\">Delete</button>';"
    "          html += '</div>';"
    "        });"
    "      }"
    "      document.getElementById('fileList').innerHTML = html;"
    "    })"
    "    .catch(e => {"
    "      document.getElementById('fileList').innerHTML = '<p>Error loading files</p>';"
    "    });"
    "}"
    "function downloadFile(name) {"
    "  window.location.href = '/download?file=' + encodeURIComponent(name);"
    "}"
    "function deleteFile(name) {"
    "  if (!confirm('Delete ' + name + '?')) return;"
    "  fetch('/api/delete?file=' + encodeURIComponent(name), {method: 'POST'})"
    "    .then(r => r.json())"
    "    .then(data => {"
    "      if (data.success) {"
    "        loadFiles();"
    "      } else {"
    "        alert('Failed to delete file');"
    "      }"
    "    });"
    "}"
    "function deleteAll() {"
    "  if (!confirm('Delete ALL files? This cannot be undone!')) return;"
    "  fetch('/api/delete-all', {method: 'POST'})"
    "    .then(r => r.json())"
    "    .then(data => {"
    "      alert('Deleted ' + data.count + ' files');"
    "      loadFiles();"
    "    })"
    "    .catch(e => alert('Failed to delete files'));"
    "}"
    "function formatFS() {"
    "  if (!confirm('Format filesystem? ALL DATA WILL BE LOST! This cannot be undone!')) return;"
    "  if (!confirm('Are you ABSOLUTELY sure? This will erase everything!')) return;"
    "  fetch('/api/format', {method: 'POST'})"
    "    .then(r => r.json())"
    "    .then(data => {"
    "      if (data.success) {"
    "        alert('Filesystem formatted successfully');"
    "        loadFiles();"
    "      } else {"
    "        alert('Failed to format filesystem');"
    "      }"
    "    })"
    "    .catch(e => alert('Format failed'));"
    "}"
    "loadFiles();"
    "</script>"
    "</body>"
    "</html>";

// Handler for root path
static esp_err_t root_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_page, strlen(html_page));
    return ESP_OK;
}

// Handler for listing files (JSON API)
static esp_err_t files_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");

    // Build JSON response with file list
    char* json = (char*)malloc(4096);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory error");
        return ESP_FAIL;
    }

    int pos = 0;
    pos += snprintf(json + pos, 4096 - pos, "{\"files\":[");

    // Use callback to iterate files
    struct {
        char* buffer;
        int* position;
        int max_size;
        bool first;
    } context = {json, &pos, 4096, true};

    auto file_callback = [](const char* filename, void* user_data) {
        auto* ctx = (decltype(context)*)user_data;

        // Skip system files
        if (filename[0] == '.') return;

        // Get file size
        char full_path[64];
        snprintf(full_path, sizeof(full_path), "/%s", filename);
        long size = fs_get_file_size(full_path);

        if (!ctx->first) {
            *ctx->position += snprintf(ctx->buffer + *ctx->position,
                                       ctx->max_size - *ctx->position, ",");
        }
        ctx->first = false;

        *ctx->position += snprintf(ctx->buffer + *ctx->position,
                                   ctx->max_size - *ctx->position,
                                   "{\"name\":\"%s\",\"size\":%ld}",
                                   filename, size);
    };

    fs_foreach_file(file_callback, &context);

    pos += snprintf(json + pos, 4096 - pos, "]}");

    httpd_resp_send(req, json, pos);
    free(json);
    return ESP_OK;
}

// Handler for downloading files
static esp_err_t download_handler(httpd_req_t *req) {
    // Get query string
    char query[128];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing file parameter");
        return ESP_FAIL;
    }

    // Extract filename from query parameter
    char filename[64];
    if (httpd_query_key_value(query, "file", filename, sizeof(filename)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid file parameter");
        return ESP_FAIL;
    }

    char full_path[72];
    snprintf(full_path, sizeof(full_path), "/%s", filename);

    ESP_LOGI(TAG, "Download request for: %s (path: %s)", filename, full_path);

    // Check if file exists
    if (!fs_file_exists(full_path)) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
        ESP_LOGE(TAG, "File not found: %s", full_path);
        return ESP_FAIL;
    }

    // Get file size
    long file_size = fs_get_file_size(full_path);
    if (file_size < 0) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to get file size");
        return ESP_FAIL;
    }

    // Allocate buffer for file content
    char* buffer = (char*)malloc(file_size);
    if (!buffer) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory error");
        return ESP_FAIL;
    }

    // Read file
    int bytes_read = fs_read_file(full_path, buffer, file_size);
    if (bytes_read < 0) {
        free(buffer);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read file");
        return ESP_FAIL;
    }

    // Set headers for download
    httpd_resp_set_type(req, "application/octet-stream");
    char disposition[128];
    snprintf(disposition, sizeof(disposition), "attachment; filename=\"%s\"", filename);
    httpd_resp_set_hdr(req, "Content-Disposition", disposition);

    // Send file content
    httpd_resp_send(req, buffer, bytes_read);
    free(buffer);

    ESP_LOGI(TAG, "Downloaded file: %s (%d bytes)", filename, bytes_read);
    return ESP_OK;
}

// Handler for deleting files
static esp_err_t delete_handler(httpd_req_t *req) {
    // Get query string
    char query[128];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing file parameter");
        return ESP_FAIL;
    }

    // Extract filename from query parameter
    char filename[64];
    if (httpd_query_key_value(query, "file", filename, sizeof(filename)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid file parameter");
        return ESP_FAIL;
    }

    char full_path[72];  // 64 + "/" + null terminator
    snprintf(full_path, sizeof(full_path), "/%s", filename);

    // Delete file
    bool success = fs_delete_file(full_path);

    // Send JSON response
    httpd_resp_set_type(req, "application/json");
    const char* response = success ? "{\"success\":true}" : "{\"success\":false}";
    httpd_resp_send(req, response, strlen(response));

    if (success) {
        ESP_LOGI(TAG, "Deleted file: %s", filename);
    } else {
        ESP_LOGE(TAG, "Failed to delete file: %s", filename);
    }

    return ESP_OK;
}

// Handler for deleting all files
static esp_err_t delete_all_handler(httpd_req_t *req) {
    int deleted_count = 0;

    // Structure to hold deletion context
    struct {
        int count;
    } context = {0};

    // Callback to delete all data files
    auto delete_callback = [](const char* filename, void* user_data) {
        auto* ctx = (decltype(context)*)user_data;

        // Skip system files
        if (filename[0] == '.') return;

        // Delete file
        char full_path[72];
        snprintf(full_path, sizeof(full_path), "/%s", filename);
        if (fs_delete_file(full_path)) {
            ctx->count++;
        }
    };

    fs_foreach_file(delete_callback, &context);

    ESP_LOGI(TAG, "Deleted all files: %d files removed", context.count);

    // Send JSON response
    httpd_resp_set_type(req, "application/json");
    char response[64];
    snprintf(response, sizeof(response), "{\"success\":true,\"count\":%d}", context.count);
    httpd_resp_send(req, response, strlen(response));

    return ESP_OK;
}

// Handler for formatting filesystem
static esp_err_t format_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "Format filesystem requested");

    bool success = fs_format();

    // Send JSON response
    httpd_resp_set_type(req, "application/json");
    const char* response = success ? "{\"success\":true}" : "{\"success\":false}";
    httpd_resp_send(req, response, strlen(response));

    if (success) {
        ESP_LOGI(TAG, "Filesystem formatted successfully");
        terminal_write("Filesystem formatted via web");
    } else {
        ESP_LOGE(TAG, "Failed to format filesystem");
    }

    return ESP_OK;
}

bool webserver_init() {
    ESP_LOGI(TAG, "Webserver initialized");
    return true;
}

bool webserver_start() {
    if (server != NULL) {
        ESP_LOGW(TAG, "Server already running");
        return true;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Starting HTTP server on port %d", config.server_port);

    if (httpd_start(&server, &config) == ESP_OK) {
        // Register URI handlers
        httpd_uri_t root_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &root_uri);

        httpd_uri_t files_uri = {
            .uri = "/api/files",
            .method = HTTP_GET,
            .handler = files_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &files_uri);

        httpd_uri_t download_uri = {
            .uri = "/download",
            .method = HTTP_GET,
            .handler = download_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &download_uri);

        httpd_uri_t delete_uri = {
            .uri = "/api/delete",
            .method = HTTP_POST,
            .handler = delete_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &delete_uri);

        httpd_uri_t delete_all_uri = {
            .uri = "/api/delete-all",
            .method = HTTP_POST,
            .handler = delete_all_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &delete_all_uri);

        httpd_uri_t format_uri = {
            .uri = "/api/format",
            .method = HTTP_POST,
            .handler = format_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &format_uri);

        ESP_LOGI(TAG, "HTTP server started");
        terminal_write("Webserver started on port 80");
        return true;
    }

    ESP_LOGE(TAG, "Failed to start HTTP server");
    terminal_write("Failed to start webserver");
    return false;
}

bool webserver_stop() {
    if (server == NULL) {
        ESP_LOGW(TAG, "Server not running");
        return true;
    }

    if (httpd_stop(server) == ESP_OK) {
        server = NULL;
        ESP_LOGI(TAG, "HTTP server stopped");
        terminal_write("Webserver stopped");
        return true;
    }

    ESP_LOGE(TAG, "Failed to stop HTTP server");
    return false;
}

bool webserver_is_running() {
    return server != NULL;
}
