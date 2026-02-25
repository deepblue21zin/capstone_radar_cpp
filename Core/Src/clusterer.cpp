// src/clusterer.cpp
/**
 * @file clusterer.cpp
 * @brief Module 2: Object Clustering & Merging
 * 
 * Responsibilities:
 * - Implement DBSCAN clustering
 * - Merge nearby detections
 * - Calculate cluster statistics
 * - Output cluster centroids for tracking
 */

#include "clusterer.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <numeric>

// ============================================================
// Constructor & Initialization
// ============================================================

Clusterer::Clusterer()
    : config_{0.8, 1, 0.3},
      cluster_count_(0),
      processing_time_ms_(0.0) {
}

bool Clusterer::Initialize() {
    std::cout << "🔧 [Module 2] Clustering initialization...\n";
    
    try {
        std::cout << "✅ [Module 2] Initialized\n";
        std::cout << "   - DBSCAN epsilon: " << config_.epsilon << " m\n";
        std::cout << "   - Min points: " << config_.min_pts << "\n\n";
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "❌ [Module 2] Initialization failed: " << e.what() << "\n";
        return false;
    }
}

// ============================================================
// Main Clustering Function
// ============================================================

ClusteringOutput Clusterer::Process(
    const std::vector<FilteredDetection>& detections,
    double timestamp) {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    ClusteringOutput output;
    output.timestamp = timestamp;
    output.input_count = detections.size();
    
    if (detections.empty()) {
        output.output_count = 0;
        output.merged_count = 0;
        output.processing_time_ms = 0.0;
        return output;
    }
    
    // ============================================================
    // Extract positions for clustering
    // ============================================================
    std::vector<Eigen::Vector3d> positions;
    for (const auto& det : detections) {
        positions.push_back(det.position);
    }
    
    // ============================================================
    // Run DBSCAN
    // ============================================================
    std::vector<int> labels = DBSCANCluster(positions, detections);
    
    // ============================================================
    // Extract and Process Clusters
    // ============================================================
    std::map<int, std::vector<int>> clusters;
    
    for (size_t i = 0; i < labels.size(); i++) {
        int label = labels[i];
        if (label >= 0) {  // Ignore noise (-1)
            clusters[label].push_back(i);
        }
    }
    
    // ============================================================
    // Compute Statistics for Each Cluster
    // ============================================================
    cluster_count_ = clusters.size();
    
    for (const auto& [cluster_id, indices] : clusters) {
        ClusteredObject obj;
        obj.cluster_id = cluster_id;
        obj.point_indices = indices;
        obj.point_count = indices.size();
        obj.timestamp = timestamp;
        
        // Compute centroid
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (int idx : indices) {
            centroid += detections[idx].position;
        }
        centroid /= indices.size();
        obj.center = centroid;
        
        // Compute average velocity
        Eigen::Vector3d avg_velocity = Eigen::Vector3d::Zero();
        for (int idx : indices) {
            avg_velocity += detections[idx].velocity;
        }
        avg_velocity /= indices.size();
        obj.velocity = avg_velocity;
        
        // Compute position covariance
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        for (int idx : indices) {
            Eigen::Vector3d diff = detections[idx].position - centroid;
            cov += diff * diff.transpose();
        }
        cov /= indices.size();
        obj.position_covariance = cov;
        
        // Compute velocity covariance
        Eigen::Matrix3d vel_cov = Eigen::Matrix3d::Zero();
        for (int idx : indices) {
            Eigen::Vector3d vel_diff = detections[idx].velocity - avg_velocity;
            vel_cov += vel_diff * vel_diff.transpose();
        }
        vel_cov /= indices.size();
        obj.velocity_covariance = vel_cov;
        
        // Quality score: SNR average + log(cluster size)
        double avg_snr = 0.0;
        for (int idx : indices) {
            avg_snr += detections[idx].snr;
        }
        avg_snr /= indices.size();
        
        double quality = (avg_snr / 30.0) + std::log(1.0 + indices.size());
        obj.quality_score = std::min(1.0, quality / 2.0);
        
        output.clusters.push_back(obj);
    }
    
    // ============================================================
    // Statistics
    // ============================================================
    output.output_count = output.clusters.size();
    output.merged_count = output.input_count - output.output_count;
    
    auto end_time = std::chrono::high_resolution_clock::now();
    output.processing_time_ms = 
        std::chrono::duration<double, std::milli>(
            end_time - start_time).count();
    processing_time_ms_ = output.processing_time_ms;
    
    return output;
}

// ============================================================
// DBSCAN Implementation
// ============================================================

std::vector<int> Clusterer::DBSCANCluster(
    const std::vector<Eigen::Vector3d>& positions,
    const std::vector<FilteredDetection>& detections) {
    
    int n = positions.size();
    std::vector<int> labels(n, -1);  // -1 = noise
    int cluster_id = 0;
    
    for (int i = 0; i < n; i++) {
        if (labels[i] != -1) {
            continue;  // Already processed
        }
        
        // Find neighbors
        std::vector<int> neighbors = RegionQuery(positions[i], positions);
        
        // Not a core point
        if ((int)neighbors.size() < config_.min_pts) {
            labels[i] = 0;  // Mark as noise
            continue;
        }
        
        // Expand cluster
        cluster_id++;
        ExpandCluster(i, neighbors, labels, positions, cluster_id);
    }
    
    return labels;
}

std::vector<int> Clusterer::RegionQuery(
    const Eigen::Vector3d& point,
    const std::vector<Eigen::Vector3d>& positions) {
    
    std::vector<int> neighbors;
    
    for (size_t j = 0; j < positions.size(); j++) {
        double dist = (point - positions[j]).norm();
        if (dist <= config_.epsilon) {
            neighbors.push_back(j);
        }
    }
    
    return neighbors;
}

void Clusterer::ExpandCluster(int idx, 
                              std::vector<int>& neighbors,
                              std::vector<int>& labels,
                              const std::vector<Eigen::Vector3d>& positions,
                              int cluster_id) {
    // Mark current point as part of cluster
    labels[idx] = cluster_id;
    
    // Seed set for expansion
    std::vector<int> seed_set = neighbors;
    
    while (!seed_set.empty()) {
        int current_idx = seed_set.back();
        seed_set.pop_back();
        
        // Mark as noise if not already assigned
        if (labels[current_idx] == -1) {
            labels[current_idx] = cluster_id;
        }
        
        // Skip if already assigned to a cluster
        if (labels[current_idx] != 0) {
            continue;
        }
        
        labels[current_idx] = cluster_id;
        
        // Find neighbors of current point
        std::vector<int> new_neighbors = 
            RegionQuery(positions[current_idx], positions);
        
        if ((int)new_neighbors.size() >= config_.min_pts) {
            seed_set.insert(seed_set.end(), 
                           new_neighbors.begin(), 
                           new_neighbors.end());
        }
    }
}

// ============================================================
// Configuration Methods
// ============================================================

void Clusterer::SetEpsilon(double epsilon) {
    config_.epsilon = epsilon;
    std::cout << "[Module 2] Epsilon updated: " << epsilon << " m\n";
}

void Clusterer::SetMinPts(int min_pts) {
    config_.min_pts = min_pts;
    std::cout << "[Module 2] MinPts updated: " << min_pts << "\n";
}

ClusteringStats Clusterer::GetStatistics() const {
    ClusteringStats stats;
    stats.cluster_count = cluster_count_;
    stats.processing_time_ms = processing_time_ms_;
    return stats;
}
