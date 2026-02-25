// src/radar_tracker.cpp
/**
 * @file radar_tracker.cpp
 * @brief Main RadarTracker - Integrated system controller
 * 
 * Coordinates all three modules:
 * - Receives raw sensor data
 * - Routes through preprocessing → clustering → tracking
 * - Manages threads and timing
 * - Provides performance monitoring
 */

#include "radar_tracker.hpp"
#include "preprocessor.hpp"
#include "clusterer.hpp"
#include "multi_object_tracker.hpp"
#include <iostream>
#include <thread>
#include <chrono>

// ============================================================
// Implementation Class
// ============================================================

class RadarTracker::Impl {
public:
    std::unique_ptr<Preprocessor> preprocessor;
    std::unique_ptr<Clusterer> clusterer;
    std::unique_ptr<MultiObjectTracker> tracker;
    
    bool initialized = false;
    bool running = false;
    
    // Performance monitoring
    std::vector<double> frame_times_ms;
    double total_preprocessing_ms = 0;
    double total_clustering_ms = 0;
    double total_tracking_ms = 0;
    int frame_count = 0;
};

// ============================================================
// Constructor & Destructor
// ============================================================

RadarTracker::RadarTracker(const std::string& config_file)
    : config_file_(config_file),
      impl_(std::make_unique<Impl>()) {
}

RadarTracker::~RadarTracker() {
    Stop();
}

// ============================================================
// Initialization
// ============================================================

bool RadarTracker::Initialize() {
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  RadarTracker System Initialization                       ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n";
    
    try {
        // Initialize all modules
        impl_->preprocessor = std::make_unique<Preprocessor>();
        if (!impl_->preprocessor->Initialize(config_file_)) {
            return false;
        }
        
        impl_->clusterer = std::make_unique<Clusterer>();
        if (!impl_->clusterer->Initialize()) {
            return false;
        }
        
        impl_->tracker = std::make_unique<MultiObjectTracker>();
        if (!impl_->tracker->Initialize()) {
            return false;
        }
        
        impl_->initialized = true;
        impl_->frame_count = 0;
        
        std::cout << "╔════════════════════════════════════════════════════════════╗\n";
        std::cout << "║  ✅ All modules initialized successfully                  ║\n";
        std::cout << "╚════════════════════════════════════════════════════════════╝\n";
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "❌ Initialization failed: " << e.what() << "\n";
        return false;
    }
}

// ============================================================
// Start/Stop
// ============================================================

void RadarTracker::Start() {
    if (!impl_->initialized) {
        std::cerr << "❌ System not initialized\n";
        return;
    }
    
    impl_->running = true;
    std::cout << "▶️  System started\n";
}

void RadarTracker::Stop() {
    impl_->running = false;
    std::cout << "⏹️  System stopped\n";
}

// ============================================================
// Main Processing
// ============================================================

std::vector<ConfirmedTrack> RadarTracker::ProcessFrame(
    const std::vector<FilteredDetection>& raw_detections,
    double timestamp) {
    
    auto frame_start = std::chrono::high_resolution_clock::now();
    
    std::vector<ConfirmedTrack> output_tracks;
    
    if (!impl_->initialized || !impl_->running) {
        return output_tracks;
    }
    
    try {
        // ============================================================
        // Module 1: Preprocessing
        // ============================================================
        auto preproc_start = std::chrono::high_resolution_clock::now();
        
        PreprocessingOutput preproc_output = 
            impl_->preprocessor->Process(raw_detections, timestamp);
        
        auto preproc_end = std::chrono::high_resolution_clock::now();
        double preproc_ms = 
            std::chrono::duration<double, std::milli>(
                preproc_end - preproc_start).count();
        impl_->total_preprocessing_ms += preproc_ms;
        
        if (preproc_output.filtered_detections.empty()) {
            impl_->frame_count++;
            return output_tracks;
        }
        
        // ============================================================
        // Module 2: Clustering
        // ============================================================
        auto cluster_start = std::chrono::high_resolution_clock::now();
        
        ClusteringOutput cluster_output = 
            impl_->clusterer->Process(preproc_output.filtered_detections, 
                                     timestamp);
        
        auto cluster_end = std::chrono::high_resolution_clock::now();
        double cluster_ms = 
            std::chrono::duration<double, std::milli>(
                cluster_end - cluster_start).count();
        impl_->total_clustering_ms += cluster_ms;
        
        if (cluster_output.clusters.empty()) {
            impl_->frame_count++;
            return output_tracks;
        }
        
        // ============================================================
        // Module 3: Tracking
        // ============================================================
        auto track_start = std::chrono::high_resolution_clock::now();
        
        TrackingOutput track_output = 
            impl_->tracker->Process(cluster_output.clusters, timestamp);
        
        auto track_end = std::chrono::high_resolution_clock::now();
        double track_ms = 
            std::chrono::duration<double, std::milli>(
                track_end - track_start).count();
        impl_->total_tracking_ms += track_ms;
        
        output_tracks = track_output.confirmed_tracks;
        
        // ============================================================
        // Frame Statistics
        // ============================================================
        auto frame_end = std::chrono::high_resolution_clock::now();
        double total_frame_ms = 
            std::chrono::duration<double, std::milli>(
                frame_end - frame_start).count();
        
        impl_->frame_times_ms.push_back(total_frame_ms);
        if (impl_->frame_times_ms.size() > 100) {
            impl_->frame_times_ms.erase(impl_->frame_times_ms.begin());
        }
        
        impl_->frame_count++;
        
        return output_tracks;
        
    } catch (const std::exception& e) {
        std::cerr << "❌ Processing error: " << e.what() << "\n";
        return output_tracks;
    }
}

// ============================================================
// Performance Metrics
// ============================================================

PerformanceMetrics RadarTracker::GetMetrics() const {
    PerformanceMetrics metrics;
    
    if (impl_->frame_count == 0) {
        return metrics;
    }
    
    // Calculate FPS
    if (!impl_->frame_times_ms.empty()) {
        double avg_frame_ms = 0;
        for (double t : impl_->frame_times_ms) {
            avg_frame_ms += t;
        }
        avg_frame_ms /= impl_->frame_times_ms.size();
        
        metrics.fps = 1000.0 / avg_frame_ms;
        metrics.average_latency_ms = avg_frame_ms;
        
        double max_latency = 0;
        for (double t : impl_->frame_times_ms) {
            max_latency = std::max(max_latency, t);
        }
        metrics.max_latency_ms = max_latency;
    }
    
    // Module statistics
    auto preproc_stats = impl_->preprocessor->GetStatistics();
    auto cluster_stats = impl_->clusterer->GetStatistics();
    auto track_stats = impl_->tracker->GetStatistics();
    
    metrics.num_confirmed_tracks = 0;  // Would be from tracker
    metrics.num_tentative_tracks = track_stats.active_tracks;
    metrics.total_detections = preproc_stats.filtered_count;
    metrics.total_clusters = cluster_stats.cluster_count;
    
    // CPU usage estimation
    double total_module_time = impl_->total_preprocessing_ms + 
                               impl_->total_clustering_ms + 
                               impl_->total_tracking_ms;
    if (impl_->frame_count > 0 && total_module_time > 0) {
        metrics.cpu_usage_percent = 
            (total_module_time / (impl_->frame_count * 33.3)) * 100.0;
    }
    
    // Memory usage (placeholder)
    metrics.memory_used_mb = 50.0;
    
    return metrics;
}

// ============================================================
// Configuration
// ============================================================

void RadarTracker::SetPreprocessingParams(double min_speed, 
                                          double max_range) {
    if (impl_->preprocessor) {
        impl_->preprocessor->SetSpeedThreshold(min_speed);
        impl_->preprocessor->SetMaxRange(max_range);
    }
}

void RadarTracker::SetClusteringParams(double epsilon, int min_pts) {
    if (impl_->clusterer) {
        impl_->clusterer->SetEpsilon(epsilon);
        impl_->clusterer->SetMinPts(min_pts);
    }
}

void RadarTracker::SetTrackingParams(int confirmation_frames,
                                     int max_missing_frames) {
    if (impl_->tracker) {
        impl_->tracker->SetConfirmationThreshold(confirmation_frames);
        impl_->tracker->SetDeletionThreshold(max_missing_frames);
    }
}
