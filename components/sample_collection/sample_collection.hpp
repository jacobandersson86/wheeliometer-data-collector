#pragma once

#include <cstdint>

/**
 * Initialize the sample collection system
 * @return true if successful, false otherwise
 */
bool sample_collection_init(void);

/**
 * Start sample collection (creates new file and starts IMU sampling)
 * @return true if successful, false otherwise
 */
bool sample_collection_start(void);

/**
 * Stop sample collection (stops sampling and closes file)
 * @return true if successful, false otherwise
 */
bool sample_collection_stop(void);

/**
 * Check if collection is active
 * @return true if collecting, false otherwise
 */
bool sample_collection_is_active(void);

/**
 * Get collection statistics
 */
struct sample_collection_stats_t {
    uint64_t total_samples_written;
    uint64_t total_bytes_written;
    uint32_t batches_written;
    uint32_t write_errors;
    uint32_t space_full_stops;
    char current_filename[64];
};

void sample_collection_get_stats(sample_collection_stats_t* stats);
