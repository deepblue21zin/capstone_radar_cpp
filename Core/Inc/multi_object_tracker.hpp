// include/multi_object_tracker.hpp
#pragma once

#include "data_types.hpp"
#include <vector>
#include <memory>
#include <chrono>

// Forward declarations
class KalmanFilter;
class HungarianMatcher;
struct Track;

/**
 * @struct TrackManagementParams
 * @brief Parameters for track confirmation/deletion
 */
struct TrackManagementParams {
    int confirmation_frames = 3;      // Frames to confirm track
    int max_consecutive_miss = 5;     // Frames to delete track
    int max_track_age = 300;          // Maximum track lifetime (frames)
};

/**
 * @struct TrackingOutput
 * @brief Output from tracking module
 */
struct TrackingOutput {
    std::vector<ConfirmedTrack> confirmed_tracks;
    std::vector<ConfirmedTrack> tentative_tracks;
    int total_active_tracks;
    double processing_time_ms;
    double timestamp;
};

/**
 * @struct TrackingStats
 * @brief Statistics from tracking
 */
struct TrackingStats {
    int active_tracks;
    double processing_time_ms;
};

/**
 * @class MultiObjectTracker
 * @brief Module 3: Multi-Object Tracking (MOT)
 * 
 * Implementations:
 * - Kalman filter for state estimation
 * - Hungarian algorithm for data association
 * - Track management state machine
 * - Confidence scoring
 */
class MultiObjectTracker {
public:
    MultiObjectTracker();
    ~MultiObjectTracker() = default;
    
    /**
     * @brief Initialize tracking module
     * @return True if successful
     */
    bool Initialize();
    
    /**
     * @brief Update tracks with new measurements
     * @param measurements Clustered objects from clustering module
     * @param timestamp Frame timestamp (seconds)
     * @return List of confirmed tracks
     */
    TrackingOutput Process(
        const std::vector<ClusteredObject>& measurements,
        double timestamp);
    
    // Configuration
    void SetConfirmationThreshold(int frames);
    void SetDeletionThreshold(int misses);
    
    // Statistics
    TrackingStats GetStatistics() const;
    
private:
    std::vector<std::unique_ptr<Track>> active_tracks_;
    std::unique_ptr<HungarianMatcher> matcher_;
    int next_track_id_;
    
    TrackManagementParams params_;
    
    int frame_count_;
    double last_timestamp_;
    double processing_time_ms_;
    
    /**
     * @brief Create new track from unmatched measurement
     */
    void CreateNewTrack(const ClusteredObject& measurement,
                       double timestamp);
};
