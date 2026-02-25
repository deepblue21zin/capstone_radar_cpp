// src/main.cpp
/**
 * @file main.cpp
 * @brief Real-Time Radar Object Tracking System
 * 
 * Entry point for the tracking system.
 * Demonstrates end-to-end pipeline from sensor to output.
 */

#include <iostream>
#include <vector>
#include <memory>
#include <chrono>
#include <iomanip>

#include "radar_tracker.hpp"
#include "data_types.hpp"

// ============================================================
// Helper Functions
// ============================================================

/**
 * @brief Print performance metrics
 */
void PrintMetrics(const PerformanceMetrics& metrics) {
    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << "📊 PERFORMANCE METRICS\n";
    std::cout << std::string(60, '=') << "\n";
    printf("├─ FPS:                    %.1f\n", metrics.fps);
    printf("├─ Avg Latency:            %.2f ms\n", metrics.average_latency_ms);
    printf("├─ Max Latency:            %.2f ms\n", metrics.max_latency_ms);
    printf("├─ Active Tracks:          %d (Confirmed: %d, Tentative: %d)\n",
           metrics.num_confirmed_tracks + metrics.num_tentative_tracks,
           metrics.num_confirmed_tracks, metrics.num_tentative_tracks);
    printf("├─ Detections (last):      %d\n", metrics.total_detections);
    printf("├─ Clusters (last):        %d\n", metrics.total_clusters);
    printf("├─ CPU Usage:              %.1f %%\n", metrics.cpu_usage_percent);
    printf("└─ Memory:                 %.1f MB\n", metrics.memory_used_mb);
    std::cout << std::string(60, '=') << "\n\n";
}

/**
 * @brief Print track information
 */
void PrintTracks(const std::vector<ConfirmedTrack>& tracks) {
    if (tracks.empty()) {
        std::cout << "ℹ️  No confirmed tracks\n";
        return;
    }
    
    std::cout << "\n📍 CONFIRMED TRACKS (" << tracks.size() << ")\n";
    std::cout << std::string(80, '-') << "\n";
    
    for (const auto& track : tracks) {
        printf("Track %3d | ", track.track_id);
        printf("Pos: [%6.2f, %6.2f, %5.2f]m | ", 
               track.position(0), track.position(1), track.position(2));
        printf("Vel: [%6.2f, %6.2f, %5.2f]m/s | ", 
               track.velocity(0), track.velocity(1), track.velocity(2));
        printf("Age: %3d | ", track.age);
        printf("Conf: %.1f%%\n", track.confidence * 100.0);
    }
    std::cout << std::string(80, '-') << "\n";
}

/**
 * @brief Generate dummy detection data for testing
 */
std::vector<FilteredDetection> GenerateDummyDetections(int num_detections, 
                                                       double timestamp) {
    std::vector<FilteredDetection> detections;
    
    // Simulate moving objects
    static int frame_count = 0;
    
    for (int i = 0; i < num_detections; i++) {
        FilteredDetection det;
        
        // Simulate object at position moving in Y direction
        double angle = (i / (double)num_detections) * 2.0 * M_PI;
        double radius = 2.0 + 0.1 * i;
        
        det.position(0) = radius * std::cos(angle);
        det.position(1) = 5.0 + 0.3 * frame_count * 0.033;  // Moving forward
        det.position(2) = 1.0;
        
        det.velocity(0) = 0.1 * std::sin(angle);
        det.velocity(1) = 0.3;  // Constant velocity in Y
        det.velocity(2) = 0.0;
        
        det.snr = 20.0 + 5.0 * (i % 3);
        det.confidence = 0.9;
        det.raw_point_id = i;
        det.position_covariance = Eigen::Matrix3d::Identity() * 0.04;
        det.state = DETECTION_VALID;
        det.timestamp = timestamp;
        
        detections.push_back(det);
    }
    
    frame_count++;
    return detections;
}

// ============================================================
// Main Application
// ============================================================

int main(int argc, char* argv[]) {
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  Real-Time Radar Object Tracking System (C++)             ║\n";
    std::cout << "║  Version: 1.0 | IWR6843ISK mmWave Radar                  ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n";
    
    try {
        // ============================================================
        // 1. Initialize Tracker
        // ============================================================
        std::cout << "\n🔧 Initializing tracker...\n";
        
        std::string config_file = "config/default_config.yaml";
        if (argc > 1) {
            config_file = argv[1];
        }
        
        std::unique_ptr<RadarTracker> tracker = 
            std::make_unique<RadarTracker>(config_file);
        
        if (!tracker->Initialize()) {
            std::cerr << "❌ Failed to initialize tracker\n";
            return 1;
        }
        
        std::cout << "✅ Tracker initialized successfully\n";
        
        // ============================================================
        // 2. Start Processing
        // ============================================================
        std::cout << "\n▶️  Starting radar tracking system...\n";
        std::cout << "Press Ctrl+C to stop\n\n";
        
        tracker->Start();
        
        // ============================================================
        // 3. Main Loop
        // ============================================================
        auto start_time = std::chrono::steady_clock::now();
        int frame_count = 0;
        const int TOTAL_FRAMES = 300;  // ~10 seconds at 30 FPS
        
        while (frame_count < TOTAL_FRAMES) {
            double timestamp = frame_count * 0.033;  // 30 FPS = 33.3ms
            
            // Generate dummy detections for testing
            // In real system: tracker_.ReceiveFromSensor(timestamp)
            auto detections = GenerateDummyDetections(50, timestamp);
            
            // Process frame
            auto frame_start = std::chrono::high_resolution_clock::now();
            auto tracks = tracker->ProcessFrame(detections, timestamp);
            auto frame_end = std::chrono::high_resolution_clock::now();
            
            double latency_ms = 
                std::chrono::duration<double, std::milli>(
                    frame_end - frame_start).count();
            
            // Print results every 30 frames (~1 second)
            if (frame_count % 30 == 0) {
                std::cout << "\n📡 Frame " << frame_count << " | "
                          << "Detections: " << detections.size() << " | "
                          << "Tracks: " << tracks.size() << " | "
                          << "Latency: " << std::fixed << std::setprecision(2) 
                          << latency_ms << "ms\n";
                
                PrintTracks(tracks);
            }
            
            frame_count++;
        }
        
        // ============================================================
        // 4. Print Final Statistics
        // ============================================================
        auto end_time = std::chrono::steady_clock::now();
        auto total_time = 
            std::chrono::duration<double>(end_time - start_time).count();
        
        auto metrics = tracker->GetMetrics();
        PrintMetrics(metrics);
        
        std::cout << "✅ Tracking completed successfully\n";
        printf("├─ Total time:    %.1f seconds\n", total_time);
        printf("├─ Total frames:  %d\n", frame_count);
        printf("└─ Effective FPS: %.1f\n\n", frame_count / total_time);
        
        // ============================================================
        // 5. Cleanup
        // ============================================================
        tracker->Stop();
        std::cout << "👋 Shutting down...\n\n";
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "❌ Exception: " << e.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "❌ Unknown error\n";
        return 1;
    }
}

/* ============================================================
   USAGE EXAMPLES
   ============================================================
   
   1. Basic run (dummy data):
      $ ./radar_tracking_main
   
   2. Run with custom config:
      $ ./radar_tracking_main config/jetson_nano.yaml
   
   3. Run with performance analysis:
      $ ./radar_tracking_main | grep "FPS:"
   
   4. Real sensor mode (when implemented):
      - Uncomment sensor initialization in tracker
      - Connect IWR6843ISK via UART
      - Data will stream automatically
   
   ============================================================ */
