// include/preprocessor.hpp
#pragma once

#include "data_types.hpp"
#include <vector>
#include <string>
#include <chrono>
#include <memory>

/**
 * @struct PreprocessingConfig
 * @brief Configuration parameters for preprocessing
 */
struct PreprocessingConfig {
    double min_speed_threshold;  // Minimum speed to consider dynamic (m/s)
    double max_range;            // Maximum detection range (m)
    double z_min, z_max;         // Height range (m)
    double x_min, x_max;         // X boundary (m)
    double y_min, y_max;         // Y boundary (m)
    double min_snr;              // Minimum signal quality (dB)
};

/**
 * @struct PreprocessingOutput
 * @brief Output from preprocessing module
 */
struct PreprocessingOutput {
    std::vector<FilteredDetection> filtered_detections;
    int raw_count;
    int filtered_count;
    double processing_time_ms;
    double timestamp;
};

/**
 * @struct PreprocessingStats
 * @brief Statistics from preprocessing
 */
struct PreprocessingStats {
    int raw_count;
    int filtered_count;
    double processing_time_ms;
};

/**
 * @class Preprocessor
 * @brief Module 1: Data Acquisition & Preprocessing
 * 
 * Handles:
 * - Sensor data reception
 * - Protocol parsing
 * - Coordinate transformation
 * - Multi-stage filtering
 */
class Preprocessor {
public:
    Preprocessor();
    ~Preprocessor() = default;
    
    /**
     * @brief Initialize preprocessing module
     * @param config_file Path to configuration file
     * @return True if successful
     */
    bool Initialize(const std::string& config_file);
    
    /**
     * @brief Process raw detections with filtering
     * @param raw_detections Raw detection list from sensor
     * @param timestamp Frame timestamp (seconds)
     * @return Filtered detections ready for clustering
     */
    PreprocessingOutput Process(
        const std::vector<FilteredDetection>& raw_detections,
        double timestamp);
    
    // Configuration setters
    void SetSpeedThreshold(double min_speed);
    void SetMaxRange(double max_range);
    void SetHeightRange(double z_min, double z_max);
    void SetBoundaryBox(double x_min, double x_max,
                        double y_min, double y_max);
    
    // Statistics
    PreprocessingStats GetStatistics() const;
    
private:
    bool initialized_;
    PreprocessingConfig config_;
    
    // Statistics tracking
    int raw_count_;
    int filtered_count_;
    double processing_time_ms_;
};
