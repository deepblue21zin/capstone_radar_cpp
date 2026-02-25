// src/multi_object_tracker.cpp
/**
 * @file multi_object_tracker.cpp
 * @brief Module 3: Multi-Object Tracking (MOT)
 * 
 * Responsibilities:
 * - Maintain track list across frames
 * - Kalman filter prediction/update
 * - Track-to-measurement association
 * - Track confirmation/deletion logic
 */

#include "multi_object_tracker.hpp"
#include "hungarian_matcher.hpp"
#include "kalman_filter.hpp"
#include <algorithm>
#include <iostream>
#include <cmath>

// ============================================================
// Track Structure Implementation
// ============================================================

struct Track {
    int track_id;
    std::unique_ptr<KalmanFilter> kalman_filter;
    
    int age;                       // Frames since creation
    int consecutive_hits;          // Frames with measurement
    int consecutive_misses;        // Frames without measurement
    int last_measurement_idx;      // Index of last associated measurement
    
    uint8_t state;                 // TENTATIVE, CONFIRMED, ABANDONED
    double confidence;
    double last_update_time;
};

// ============================================================
// Constructor & Initialization
// ============================================================

MultiObjectTracker::MultiObjectTracker()
    : next_track_id_(1),
      frame_count_(0),
      processing_time_ms_(0.0) {
    
    // Default parameters
    params_.confirmation_frames = 3;
    params_.max_consecutive_miss = 5;
    params_.max_track_age = 300;      // ~10 seconds at 30 FPS
}

bool MultiObjectTracker::Initialize() {
    std::cout << "🔧 [Module 3] Tracking initialization...\n";
    
    try {
        matcher_ = std::make_unique<HungarianMatcher>();
        
        std::cout << "✅ [Module 3] Initialized\n";
        std::cout << "   - Confirmation frames: " 
                  << params_.confirmation_frames << "\n";
        std::cout << "   - Max consecutive miss: " 
                  << params_.max_consecutive_miss << "\n";
        std::cout << "   - Max track age: " 
                  << params_.max_track_age << "\n\n";
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "❌ [Module 3] Initialization failed: " << e.what() << "\n";
        return false;
    }
}

// ============================================================
// Main Tracking Update
// ============================================================

TrackingOutput MultiObjectTracker::Process(
    const std::vector<ClusteredObject>& measurements,
    double timestamp) {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    TrackingOutput output;
    output.timestamp = timestamp;
    
    frame_count_++;
    
    // ============================================================
    // Step 1: Predict - Update all tracks with Kalman filter
    // ============================================================
    double dt = 0.033;  // 30 FPS
    if (frame_count_ > 1) {
        dt = timestamp - last_timestamp_;
    }
    
    for (auto& track : active_tracks_) {
        track->kalman_filter->Predict(dt);
    }
    
    // ============================================================
    // Step 2: Association - Match tracks to measurements
    // ============================================================
    
    std::vector<int> track_to_measurement(active_tracks_.size(), -1);
    std::vector<bool> measurement_used(measurements.size(), false);
    
    if (!active_tracks_.empty() && !measurements.empty()) {
        // Compute cost matrix using Mahalanobis distances
        Eigen::MatrixXd cost_matrix(active_tracks_.size(), 
                                    measurements.size());
        
        for (size_t i = 0; i < active_tracks_.size(); i++) {
            for (size_t j = 0; j < measurements.size(); j++) {
                double dist = active_tracks_[i]->kalman_filter->
                    MahalanobisDistance(measurements[j].center);
                cost_matrix(i, j) = dist;
            }
        }
        
        // Solve assignment problem
        track_to_measurement = matcher_->Solve(cost_matrix);
        
        // Mark used measurements
        for (int meas_idx : track_to_measurement) {
            if (meas_idx >= 0) {
                measurement_used[meas_idx] = true;
            }
        }
    }
    
    // ============================================================
    // Step 3: Update - Kalman filter update for matched tracks
    // ============================================================
    
    for (size_t i = 0; i < active_tracks_.size(); i++) {
        int meas_idx = track_to_measurement[i];
        
        if (meas_idx >= 0) {
            // Measurement found - update filter
            active_tracks_[i]->kalman_filter->
                Update(measurements[meas_idx].center);
            
            active_tracks_[i]->consecutive_hits++;
            active_tracks_[i]->consecutive_misses = 0;
            active_tracks_[i]->last_measurement_idx = meas_idx;
            
            // Update confidence
            active_tracks_[i]->confidence = 
                std::min(1.0, active_tracks_[i]->confidence + 0.1);
        } else {
            // No measurement - only use prediction
            active_tracks_[i]->consecutive_misses++;
            
            // Reduce confidence
            active_tracks_[i]->confidence = 
                std::max(0.0, active_tracks_[i]->confidence - 0.15);
        }
        
        active_tracks_[i]->age++;
    }
    
    // ============================================================
    // Step 4: Create new tracks from unmatched measurements
    // ============================================================
    
    for (size_t j = 0; j < measurements.size(); j++) {
        if (!measurement_used[j]) {
            CreateNewTrack(measurements[j], timestamp);
        }
    }
    
    // ============================================================
    // Step 5: Track management - Confirm/Delete
    // ============================================================
    
    std::vector<size_t> tracks_to_remove;
    
    for (size_t i = 0; i < active_tracks_.size(); i++) {
        auto& track = active_tracks_[i];
        
        // State transitions
        if (track->state == TRACK_TENTATIVE) {
            // Try to confirm
            if (track->age >= params_.confirmation_frames &&
                track->consecutive_hits >= params_.confirmation_frames) {
                track->state = TRACK_CONFIRMED;
                // std::cout << "✓ Track " << track->track_id 
                //           << " confirmed\n";
            }
        }
        
        // Delete old or lost tracks
        if (track->consecutive_misses > params_.max_consecutive_miss ||
            track->age > params_.max_track_age) {
            track->state = TRACK_ABANDONED;
            tracks_to_remove.push_back(i);
        }
    }
    
    // Remove abandoned tracks (in reverse order to maintain indices)
    for (int i = (int)tracks_to_remove.size() - 1; i >= 0; i--) {
        active_tracks_.erase(active_tracks_.begin() + tracks_to_remove[i]);
    }
    
    // ============================================================
    // Step 6: Output
    // ============================================================
    
    for (const auto& track : active_tracks_) {
        if (track->state == TRACK_CONFIRMED) {
            ConfirmedTrack out_track;
            out_track.track_id = track->track_id;
            out_track.position = track->kalman_filter->GetPosition();
            out_track.velocity = track->kalman_filter->GetVelocity();
            out_track.acceleration = track->kalman_filter->GetAcceleration();
            out_track.position_covariance = 
                track->kalman_filter->GetPositionCovariance();
            
            out_track.age = track->age;
            out_track.consecutive_hits = track->consecutive_hits;
            out_track.consecutive_misses = track->consecutive_misses;
            out_track.confidence = track->confidence;
            out_track.state = TRACK_CONFIRMED;
            out_track.timestamp = timestamp;
            out_track.object_class = CLASS_UNKNOWN;
            
            output.confirmed_tracks.push_back(out_track);
        }
    }
    
    // ============================================================
    // Statistics
    // ============================================================
    
    auto end_time = std::chrono::high_resolution_clock::now();
    output.processing_time_ms = 
        std::chrono::duration<double, std::milli>(
            end_time - start_time).count();
    processing_time_ms_ = output.processing_time_ms;
    
    output.total_active_tracks = active_tracks_.size();
    last_timestamp_ = timestamp;
    
    return output;
}

// ============================================================
// Private Methods
// ============================================================

void MultiObjectTracker::CreateNewTrack(
    const ClusteredObject& measurement,
    double timestamp) {
    
    auto new_track = std::make_unique<Track>();
    new_track->track_id = next_track_id_++;
    new_track->kalman_filter = std::make_unique<KalmanFilter>();
    
    new_track->kalman_filter->Initialize(measurement.center, 
                                        measurement.velocity);
    
    new_track->age = 1;
    new_track->consecutive_hits = 1;
    new_track->consecutive_misses = 0;
    new_track->last_measurement_idx = -1;
    new_track->state = TRACK_TENTATIVE;
    new_track->confidence = 0.3;
    new_track->last_update_time = timestamp;
    
    // std::cout << "+ Track " << new_track->track_id << " created\n";
    
    active_tracks_.push_back(std::move(new_track));
}

// ============================================================
// Configuration
// ============================================================

void MultiObjectTracker::SetConfirmationThreshold(int frames) {
    params_.confirmation_frames = frames;
    std::cout << "[Module 3] Confirmation threshold: " << frames << " frames\n";
}

void MultiObjectTracker::SetDeletionThreshold(int misses) {
    params_.max_consecutive_miss = misses;
    std::cout << "[Module 3] Deletion threshold: " << misses << " misses\n";
}

TrackingStats MultiObjectTracker::GetStatistics() const {
    TrackingStats stats;
    stats.active_tracks = active_tracks_.size();
    stats.processing_time_ms = processing_time_ms_;
    return stats;
}
