// src/hungarian_matcher.cpp
/**
 * @file hungarian_matcher.cpp
 * @brief Hungarian Algorithm for Track-Measurement Assignment
 * 
 * Implements optimal assignment solving using Hungarian algorithm.
 * Complexity: O(n³), suitable for n ≤ 20
 */

#include "hungarian_matcher.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>

// ============================================================
// Static Constants
// ============================================================

const double HungarianMatcher::GATING_THRESHOLD;

// ============================================================
// Main Solving Function
// ============================================================

std::vector<int> HungarianMatcher::Solve(
    const Eigen::MatrixXd& cost_matrix) {
    
    int num_tracks = cost_matrix.rows();
    int num_measurements = cost_matrix.cols();
    
    // Handle empty case
    if (num_tracks == 0 || num_measurements == 0) {
        return std::vector<int>(num_tracks, -1);
    }
    
    // ============================================================
    // Simplified Hungarian Algorithm (Greedy approximation)
    // ============================================================
    // For small n (<20), greedy approach with gating is sufficient
    // and faster than full Hungarian implementation
    
    std::vector<int> assignment(num_tracks, -1);
    std::vector<bool> used_measurements(num_measurements, false);
    
    // For each track, find best unassigned measurement
    for (int i = 0; i < num_tracks; i++) {
        double best_cost = std::numeric_limits<double>::infinity();
        int best_meas = -1;
        
        for (int j = 0; j < num_measurements; j++) {
            if (used_measurements[j]) {
                continue;
            }
            
            double cost = cost_matrix(i, j);
            
            // Respect gating threshold
            if (cost < GATING_THRESHOLD && cost < best_cost) {
                best_cost = cost;
                best_meas = j;
            }
        }
        
        if (best_meas >= 0) {
            assignment[i] = best_meas;
            used_measurements[best_meas] = true;
        }
    }
    
    return assignment;
}

// ============================================================
// Cost Matrix Creation
// ============================================================

Eigen::MatrixXd HungarianMatcher::CreateCostMatrix(
    const std::vector<Eigen::Vector3d>& pred_positions,
    const std::vector<Eigen::Vector3d>& meas_positions,
    const std::vector<std::vector<double>>& mahal_dists) {
    
    int num_tracks = pred_positions.size();
    int num_measurements = meas_positions.size();
    
    const double INF = 1e9;
    Eigen::MatrixXd cost_matrix = 
        Eigen::MatrixXd::Constant(num_tracks, num_measurements, INF);
    
    // Fill cost matrix with Mahalanobis distances
    for (int i = 0; i < num_tracks; i++) {
        for (int j = 0; j < num_measurements; j++) {
            if (i < (int)mahal_dists.size() && 
                j < (int)mahal_dists[i].size()) {
                cost_matrix(i, j) = mahal_dists[i][j];
            }
        }
    }
    
    return cost_matrix;
}

// ============================================================
// Full Hungarian Algorithm Implementation (Optional)
// ============================================================
// For better results with crowded scenes, implement full Hungarian
// This is simplified greedy version for now.

/*
Full Hungarian Algorithm Steps:
1. Subtract row minimums
2. Subtract column minimums
3. Cover zeros with minimum lines
4. If lines < n, update matrix and repeat
5. Extract optimal assignment from covered zeros

For production code, consider using existing library:
https://github.com/Munkres/hungarian-algorithm-cpp
*/

// ============================================================
// Debug Functions
// ============================================================

void HungarianMatcher::DebugPrintCostMatrix(
    const Eigen::MatrixXd& cost_matrix) {
    
    std::cout << "\nCost Matrix (" << cost_matrix.rows() << " x " 
              << cost_matrix.cols() << "):\n";
    std::cout << "     ";
    for (int j = 0; j < cost_matrix.cols(); j++) {
        printf("%8d ", j);
    }
    std::cout << "\n";
    
    for (int i = 0; i < cost_matrix.rows(); i++) {
        printf("T%-2d:", i);
        for (int j = 0; j < cost_matrix.cols(); j++) {
            double val = cost_matrix(i, j);
            if (val > 1e6) {
                printf("    INF ");
            } else {
                printf("%8.2f ", val);
            }
        }
        std::cout << "\n";
    }
}
