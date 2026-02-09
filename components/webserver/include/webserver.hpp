#pragma once
#include <stdbool.h>

// Initialize webserver subsystem
// Must be called before start/stop functions
bool webserver_init();

// Start the HTTP server
// Returns true on success
bool webserver_start();

// Stop the HTTP server
// Returns true on success
bool webserver_stop();

// Check if server is running
bool webserver_is_running();
