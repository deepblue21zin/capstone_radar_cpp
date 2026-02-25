// src/preprocessor.cpp
/**
 * @file preprocessor.cpp
 * @brief Module 1: Data Acquisition & Preprocessing
 * 
 * Responsibilities:
 * - Receive raw radar data from sensor
 * - Parse binary protocol
 * - Filter by speed, range, height, SNR
 * - Apply coordinate transforms
 * - Output clean detection list for clustering
 */

#include "preprocessor.hpp"
#include <cmath>
#include <numeric>
#include <algorithm>
#include <iostream>

// ============================================================
// Constructor & Initialization
// ============================================================

Preprocessor::Preprocessor()
    : initialized_(false),
      raw_count_(0),
      filtered_count_(0),
      processing_time_ms_(0.0) {
    
    // Default configuration
    config_.min_speed_threshold = 0.2;
    config_.max_range = 12.0;
    config_.z_min = 0.0;
    config_.z_max = 2.5;
    config_.x_min = -2.0;
    config_.x_max = 2.0;
    config_.y_min = 0.5;
    config_.y_max = 8.0;
    config_.min_snr = 10.0;
}

bool Preprocessor::Initialize(const std::string& config_file) {
    std::cout << "🔧 [Module 1] Preprocessing initialization...\n";
    
    try {
        // TODO: Parse config file (YAML/JSON)
        // For now, use default values
        
        initialized_ = true;
        
        std::cout << "✅ [Module 1] Initialized\n";
        std::cout << "   - Speed threshold: " << config_.min_speed_threshold 
                  << " m/s\n";
        std::cout << "   - Max range: " << config_.max_range << " m\n";
        std::cout << "   - Height range: [" << config_.z_min << ", " 
                  << config_.z_max << "] m\n";
        std::cout << "   - X range: [" << config_.x_min << ", " 
                  << config_.x_max << "] m\n";
        std::cout << "   - Y range: [" << config_.y_min << ", " 
                  << config_.y_max << "] m\n\n";
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "❌ [Module 1] Initialization failed: " << e.what() << "\n";
        return false;
    }
}

// ============================================================
// Main Processing Function
// ============================================================

PreprocessingOutput Preprocessor::Process(
    const std::vector<FilteredDetection>& raw_detections,
    double timestamp) {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    PreprocessingOutput output;
    output.timestamp = timestamp;
    output.raw_count = raw_detections.size();
    
    if (!initialized_) {
        output.processing_time_ms = 0.0;
        output.filtered_count = 0;
        return output;
    }
    
    // ============================================================
    // Filtering Pipeline
    // ============================================================
    
    for (const auto& det : raw_detections) {
        // 1. Speed filter
        if (det.velocity.norm() < config_.min_speed_threshold) {
            continue;  // Static object - skip
        }
        
        // 2. Range filter
        double range = det.position.norm();
        if (range > config_.max_range) {
            continue;  // Too far
        }
        
        // 3. Height filter
        if (det.position(2) < config_.z_min || 
            det.position(2) > config_.z_max) {
            continue;  // Outside height range
        }
        
        // 4. SNR filter
        if (det.snr < config_.min_snr) {
            continue;  // Low signal quality
        }
        
        // 5. Boundary box filter (X, Y)
        if (det.position(0) < config_.x_min || 
            det.position(0) > config_.x_max) {
            continue;  // Outside X range
        }
        
        if (det.position(1) < config_.y_min || 
            det.position(1) > config_.y_max) {
            continue;  // Outside Y range
        }
        
        // Passed all filters
        output.filtered_detections.push_back(det);
    }
    
    // ============================================================
    // Statistics
    // ============================================================
    
    output.filtered_count = output.filtered_detections.size();
    raw_count_ = output.raw_count;
    filtered_count_ = output.filtered_count;
    
    auto end_time = std::chrono::high_resolution_clock::now();
    output.processing_time_ms = 
        std::chrono::duration<double, std::milli>(
            end_time - start_time).count();
    processing_time_ms_ = output.processing_time_ms;
    
    return output;
}

// ============================================================
// Configuration Methods
// ============================================================

void Preprocessor::SetSpeedThreshold(double min_speed) {
    config_.min_speed_threshold = min_speed;
    std::cout << "[Module 1] Speed threshold updated: " << min_speed << " m/s\n";
}

void Preprocessor::SetMaxRange(double max_range) {
    config_.max_range = max_range;
    std::cout << "[Module 1] Max range updated: " << max_range << " m\n";
}

void Preprocessor::SetHeightRange(double z_min, double z_max) {
    config_.z_min = z_min;
    config_.z_max = z_max;
    std::cout << "[Module 1] Height range updated: [" << z_min << ", " 
              << z_max << "] m\n";
}

void Preprocessor::SetBoundaryBox(double x_min, double x_max,
                                  double y_min, double y_max) {
    config_.x_min = x_min;
    config_.x_max = x_max;
    config_.y_min = y_min;
    config_.y_max = y_max;
    std::cout << "[Module 1] Boundary box updated: "
              << "X=[" << x_min << ", " << x_max << "], "
              << "Y=[" << y_min << ", " << y_max << "] m\n";
}

PreprocessingStats Preprocessor::GetStatistics() const {
    PreprocessingStats stats;
    stats.raw_count = raw_count_;
    stats.filtered_count = filtered_count_;
    stats.processing_time_ms = processing_time_ms_;
    return stats;
}
