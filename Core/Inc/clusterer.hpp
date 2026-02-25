// include/clusterer.hpp
#pragma once

#include "data_types.hpp"
#include <vector>
#include <string>
#include <map>
#include <chrono>

/**
 * @struct ClusteringConfig
 * @brief DBSCAN parameters
 */
struct ClusteringConfig {
    double epsilon;      // Neighborhood radius (m)
    int min_pts;         // Minimum points for core point
    double quality_weight;
};

/**
 * @struct ClusteringOutput
 * @brief Output from clustering module
 */
struct ClusteringOutput {
    std::vector<ClusteredObject> clusters;
    int input_count;
    int output_count;
    int merged_count;
    double processing_time_ms;
    double timestamp;
};

/**
 * @struct ClusteringStats
 * @brief Statistics from clustering
 */
struct ClusteringStats {
    int cluster_count;
    double processing_time_ms;
};

/**
 * @class Clusterer
 * @brief Module 2: Object Clustering & Merging
 * 
 * Implements DBSCAN algorithm for merging nearby detections.
 * Uses optimized regional query for O(n log n) performance.
 */
class Clusterer {
public:
    Clusterer();
    ~Clusterer() = default;
    
    /**
     * @brief Initialize clustering module
     * @return True if successful
     */
    bool Initialize();
    
    /**
     * @brief Cluster filtered detections
     * @param detections Filtered detection list from preprocessing
     * @param timestamp Frame timestamp (seconds)
     * @return Clustered objects ready for tracking
     */
    ClusteringOutput Process(
        const std::vector<FilteredDetection>& detections,
        double timestamp);
    
    // Configuration
    void SetEpsilon(double epsilon);
    void SetMinPts(int min_pts);
    
    // Statistics
    ClusteringStats GetStatistics() const;
    
private:
    ClusteringConfig config_;
    
    /**
     * @brief Run DBSCAN clustering algorithm
     * @param positions Point coordinates
     * @param detections Associated detection data
     * @return Cluster labels (-1 = noise, 0+ = cluster_id)
     */
    std::vector<int> DBSCANCluster(
        const std::vector<Eigen::Vector3d>& positions,
        const std::vector<FilteredDetection>& detections);
    
    /**
     * @brief Find neighbors within epsilon radius
     */
    std::vector<int> RegionQuery(
        const Eigen::Vector3d& point,
        const std::vector<Eigen::Vector3d>& positions);
    
    /**
     * @brief Expand cluster during DBSCAN
     */
    void ExpandCluster(int idx,
                      std::vector<int>& neighbors,
                      std::vector<int>& labels,
                      const std::vector<Eigen::Vector3d>& positions,
                      int cluster_id);
    
    // Statistics
    int cluster_count_;
    double processing_time_ms_;
};
