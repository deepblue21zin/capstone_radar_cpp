// include/radar_tracker.hpp
#pragma once

#include "data_types.hpp"
#include <vector>
#include <string>
#include <memory>
#include <chrono>

/**
 * @class RadarTracker
 * @brief Main system controller integrating all three modules
 * 
 * Coordinates the complete pipeline:
 * Module 1 (Preprocessing) → Module 2 (Clustering) → Module 3 (Tracking)
 * 
 * Usage:
 * @code
 * RadarTracker tracker("config/default.yaml");
 * tracker.Initialize();
 * tracker.Start();
 * 
 * while (running) {
 *     auto detections = sensor.GetFrame();
 *     auto tracks = tracker.ProcessFrame(detections, timestamp);
 *     for (const auto& t : tracks) {
 *         printf("Track %d: %.2f, %.2f\n", t.track_id, t.position.x(), 
 *               t.position.y());
 *     }
 * }
 * tracker.Stop();
 * @endcode
 */
class RadarTracker {
public:
    /**
     * @brief Constructor
     * @param config_file Path to configuration file (YAML/JSON)
     */
    explicit RadarTracker(const std::string& config_file);
    
    /**
     * @brief Destructor - cleans up resources
     */
    ~RadarTracker();
    
    /**
     * @brief Initialize all modules
     * @return True if all modules initialized successfully
     */
    bool Initialize();
    
    /**
     * @brief Start the tracking system
     */
    void Start();
    
    /**
     * @brief Stop the tracking system
     */
    void Stop();
    
    /**
     * @brief Process a single frame
     * @param raw_detections Raw detection points from sensor
     * @param timestamp Frame timestamp in seconds
     * @return List of confirmed tracks
     */
    std::vector<ConfirmedTrack> ProcessFrame(
        const std::vector<FilteredDetection>& raw_detections,
        double timestamp);
    
    /**
     * @brief Get system performance metrics
     * @return Performance statistics (FPS, latency, etc.)
     */
    PerformanceMetrics GetMetrics() const;
    
    // ============================================================
    // Configuration Methods
    // ============================================================
    
    /**
     * @brief Configure preprocessing module
     * @param min_speed Minimum speed threshold (m/s)
     * @param max_range Maximum detection range (m)
     */
    void SetPreprocessingParams(double min_speed, double max_range);
    
    /**
     * @brief Configure clustering module
     * @param epsilon DBSCAN epsilon (m)
     * @param min_pts DBSCAN minimum points
     */
    void SetClusteringParams(double epsilon, int min_pts);
    
    /**
     * @brief Configure tracking module
     * @param confirmation_frames Frames to confirm track
     * @param max_missing_frames Max frames without measurement
     */
    void SetTrackingParams(int confirmation_frames,
                          int max_missing_frames);
    
private:
    std::string config_file_;
    
    // Implementation (Pimpl pattern for hiding complex internals)
    class Impl;
    std::unique_ptr<Impl> impl_;
};

/* ============================================================
   SYSTEM ARCHITECTURE
   ============================================================
   
   Input: Raw Radar Detections (100-1000 points/frame)
       ↓
   [Module 1: Preprocessing]
   - Filter by speed, range, height, SNR
   - Output: 30-100 filtered detections
       ↓
   [Module 2: Clustering]
   - DBSCAN spatial clustering
   - Output: 5-30 clustered objects
       ↓
   [Module 3: Tracking]
   - Kalman filter + Hungarian assignment
   - Output: 0-20 confirmed tracks
       ↓
   Result: Persistent tracked objects with ID, position, velocity
   
   Performance Target: 30+ FPS (<33ms latency)
   
   ============================================================ */
