# Real-Time Radar Object Tracking System - C++ Implementation Guide

## Executive Summary

### Current Issues Analysis
- **MATLAB Performance**: 0.8 FPS → **37x slower** than real-time requirement
- **Latency**: ~1250ms/frame vs 33ms target (30fps requirement)
- **Root Causes**:
  1. MATLAB interpreter overhead
  2. Dynamic memory allocation per frame
  3. Library wrapper inefficiencies
  4. Single-threaded execution

### Proposed Solutions
1. **C++ Native Implementation** (37x performance gain expected)
2. **Custom Kalman Filter** + **Self-implemented JPDA** (no more black-box libraries)
3. **Optimized DBSCAN** with KD-Tree spatial indexing
4. **Real-time architecture**: Ring buffers, memory pools, multi-threading

### Key Metrics Target
| Metric | Current (MATLAB) | Target (C++) |
|--------|------------------|-------------|
| **FPS** | 0.8 | **30+** |
| **Latency** | 1250ms | **<33ms** |
| **Detections/frame** | 100-1000 raw | 30-100 filtered |
| **Max tracks** | - | 20 confirmed |
| **CPU Usage** | 90%+ | <50% |

---

## I. SYSTEM ARCHITECTURE

### 1.1 Data Flow Pipeline

```
mmWave Radar (IWR6843ISK)
    ↓ [UART/USB, 100K+ points/sec]
┌─────────────────────────────────────┐
│ Module 1: PREPROCESSING             │
│ • Raw data parsing & filtering      │
│ • Coordinate transform              │
│ • Speed/Range/Height filtering      │
│ • Boundary Box clipping             │
│ Output: 30-100 filtered points      │
└────────────┬────────────────────────┘
             ↓
┌─────────────────────────────────────┐
│ Module 2: CLUSTERING                │
│ • DBSCAN spatial clustering         │
│ • Point merging (avg position/vel)  │
│ • Covariance calculation            │
│ Output: 5-30 clustered objects      │
└────────────┬────────────────────────┘
             ↓
┌─────────────────────────────────────┐
│ Module 3: TRACKING (MOT)            │
│ • Kalman filter prediction          │
│ • Hungarian/JPDA association        │
│ • Track confirmation/deletion       │
│ Output: 0-20 confirmed tracks       │
└────────────┬────────────────────────┘
             ↓
        Track List (ID, Pos, Vel)
        Visualization / Logging
```

### 1.2 System Specifications

| Component | Specification |
|-----------|---------------|
| **Sensor** | TI IWR6843ISK (60GHz mmWave) |
| **Processing Frequency** | 30+ FPS (33.3ms/frame) |
| **Max Latency** | 100ms (end-to-end) |
| **Max Objects** | 20 concurrent tracks |
| **Tracking Range** | 0.5m ~ 12m |
| **Max Velocity** | ±50 m/s (180 km/h) |
| **Implementation** | C++17 |
| **Target Platform** | Jetson Nano / Xavier, Windows |

### 1.3 Performance Bottleneck Analysis

#### Current System Profiling (MATLAB)
```
Frame Time Analysis (1000ms total):
┌──────────────────────────────────────────────────┐
│ Data Reception     │ 50ms  │ 5%  │ ◼◼              │
│ Preprocessing      │ 200ms │ 20% │ ◼◼◼◼◼           │
│ DBSCAN Clustering  │ 300ms │ 30% │ ◼◼◼◼◼◼◼         │
│ JPDA Tracking      │ 400ms │ 40% │ ◼◼◼◼◼◼◼◼◼◼     │
│ Visualization      │ 50ms  │ 5%  │ ◼◼              │
└──────────────────────────────────────────────────┘
Total: 1000ms/frame = 1 FPS (MATLAB interpreted)
```

#### Optimized C++ Target
```
Frame Time Analysis (30ms target):
┌──────────────────────────────────────────────────┐
│ Preprocessing      │ 7ms  │ 21% │ ◼◼◼             │
│ Clustering         │ 5ms  │ 15% │ ◼◼              │
│ Tracking           │ 8ms  │ 24% │ ◼◼◼             │
│ Association        │ 5ms  │ 15% │ ◼◼              │
│ Utilities (10%)    │ 3ms  │ 9%  │ ◼               │
│ Margin (6%)        │ 2ms  │ 6%  │ ◼               │
└──────────────────────────────────────────────────┘
Total: 30ms/frame = 33.3 FPS (optimized C++)
Speedup: ~33x
```

---

## II. MODULE SPECIFICATIONS

### Module 1: DATA ACQUISITION & PREPROCESSING

#### 2.1 Purpose
Filter raw radar measurements and prepare reliable detection points for clustering.

#### 2.2 Input/Output Specification

**Input:**
```cpp
struct RawDetection {
    double x, y, z;              // Position (m)
    double vx, vy, vz;           // Velocity (m/s)
    uint16_t range_idx;          // Range bin index
    int16_t doppler_idx;         // Doppler bin index
    double snr;                  // Signal-to-Noise Ratio (dB)
    uint8_t elevation;           // Elevation angle (encoded)
    double timestamp;
};
```

**Output:**
```cpp
struct FilteredDetection {
    Eigen::Vector3d position;           // (x, y, z) after filtering
    Eigen::Vector3d velocity;           // (vx, vy, vz)
    double snr;                        // Signal strength
    double confidence;                 // 0.0 ~ 1.0
    uint32_t raw_point_id;            // Original point ID
    Eigen::Matrix3d position_covariance; // Measurement uncertainty
    uint8_t state;                     // VALID, OUTLIER, MISSING
};

struct PreprocessingOutput {
    std::vector<FilteredDetection> detections;
    int raw_count;
    int filtered_count;
    double processing_time_ms;
    double timestamp;
};
```

#### 2.3 Algorithm Specification

##### 2.3.1 Raw Data Reception

**Hardware Interface:**
- Protocol: TI mmWave binary format (UART/USB)
- Baud Rate: 921600 bps
- Frame Structure:
  ```
  [Sync Word: 0x0102030405060708 (8B)]
  [Version (4B)] [Length (4B)]
  [Message Type (TLV1...TLVn)]
  [CRC-32 (4B)]
  ```

**Implementation Priority (C++):**
1. Ring Buffer for zero-copy data streaming
2. Producer thread for continuous reading
3. Consumer thread for parsing

```cpp
class RadarDataReceiver {
private:
    const size_t BUFFER_SIZE = 65536;  // 64KB ring buffer
    RingBuffer<uint8_t> rx_buffer_;
    std::thread rx_thread_;
    std::atomic<bool> running_;
    
public:
    void Start();
    std::vector<RawDetection> GetNewDetections();
};
```

##### 2.3.2 Coordinate Transform

```
Input Measurement Space:
  - x, y: Sensor Rectangular (m)
  - z: Elevation (m)
  - Range Bin Index
  - Doppler Bin Index

Output Detection Space:
  - Position: (x, y, z) in meters
  - Velocity: (vx, vy, vz) from Doppler
  - Covariance: Position uncertainty
```

**Transform Equations:**
```
Range (m) = sqrt(x² + y² + z²)
Azimuth (rad) = atan2(x, y)
Elevation (rad) = atan2(z, sqrt(x² + y²))

Doppler Velocity = (doppler_idx / FFT_size) * 
                   range_resolution * c / (2 * f_carrier)

Position Covariance:
  σ_pos = [range_resolution / sqrt(12), 
           range_resolution / sqrt(12),
           elevation_resolution / sqrt(12)]

  Cov = diag([σ_pos_x², σ_pos_y², σ_pos_z²])
```

##### 2.3.3 Filtering Pipeline

**Cascaded Filtering Strategy:**

| Filter Stage | Condition | Threshold | Impact |
|-------------|-----------|-----------|--------|
| **Speed Filter** | `norm([vx, vy, vz]) > min_speed` | 0.2 m/s | Remove static objects |
| **Range Filter** | `sqrt(x²+y²+z²) ≤ max_range` | 12 m | Ignore distant noise |
| **Height Filter** | `z_min ≤ z ≤ z_max` | 0~2.5 m | Vehicle height |
| **SNR Filter** | `snr ≥ min_snr` | 10 dB | Signal quality |
| **ROI Filter** | `(x_min≤x≤x_max) ∧ (y_min≤y≤y_max)` | ±2m, 0.5~8m | Interest area |
| **Elevation Filter** | `-5° ≤ elev ≤ 5°` | ±5° | Ignore ceiling/ground |

**Pseudocode:**
```cpp
for (const auto& raw_det : raw_detections) {
    // Early rejection
    if (raw_det.speed < 0.2) continue;        // Static
    if (raw_det.range > 12.0) continue;        // Too far
    if (raw_det.snr < 10) continue;            // Weak signal
    
    // Geometric filtering
    if (raw_det.x < -2.0 || raw_det.x > 2.0) continue;
    if (raw_det.y < 0.5 || raw_det.y > 8.0) continue;
    if (raw_det.z < 0.0 || raw_det.z > 2.5) continue;
    
    // Passed all filters
    filtered_detections.push_back(ApplyTransform(raw_det));
}
```

#### 2.4 Performance Optimization

**Memory Optimization:**
```cpp
// Pre-allocate buffers
class PreprocessorImpl {
private:
    std::vector<RawDetection> raw_pool_;        // Size: 1000
    std::vector<FilteredDetection> filtered_pool_; // Size: 200
    size_t raw_idx_, filtered_idx_;
    
public:
    void ResetBuffers() {
        raw_idx_ = 0;
        filtered_idx_ = 0;
    }
};
```

**Parallelization:**
```cpp
// Thread 1: Reception (continuous)
std::thread receiver([this]() {
    while (running_) {
        std::vector<uint8_t> data = serial_.Read();
        rx_buffer_.Push(data);
    }
});

// Thread 2: Processing (with buffer)
std::thread processor([this]() {
    while (running_) {
        auto data = rx_buffer_.Pop();
        auto filtered = Process(data);
        output_queue_.Push(filtered);
    }
});
```

**SIMD Optimization (ARM NEON):**
```cpp
// Vectorized distance calculation
float32x4_t x_v = vld1q_f32(x_array);
float32x4_t y_v = vld1q_f32(y_array);
float32x4_t z_v = vld1q_f32(z_array);

// range = sqrt(x² + y² + z²)
float32x4_t ranges = vsqrtq_f32(
    vaddq_f32(
        vaddq_f32(vmulq_f32(x_v, x_v), vmulq_f32(y_v, y_v)),
        vmulq_f32(z_v, z_v)
    )
);
```

#### 2.5 Implementation Checklist

- [ ] **SerialPort Class** with configurable baud rate
- [ ] **RingBuffer** lock-free data structure
- [ ] **TI Protocol Parser** for frame decoding
- [ ] **Coordinate Transform** with vectorization
- [ ] **Cascaded Filtering** pipeline
- [ ] **Performance Monitoring** (frame rate, latency)
- [ ] **Error Handling** (checksum, timeout)
- [ ] **Unit Tests** (N random frames)

---

### Module 2: OBJECT CLUSTERING & MERGING

#### 3.1 Purpose
Merge spatially-close detections into single object measurements for tracking.

#### 3.2 Input/Output Specification

**Input:**
```cpp
std::vector<FilteredDetection> filtered_detections;  // 30-100 points
```

**Output:**
```cpp
struct ClusteredObject {
    int cluster_id;                         // Unique ID in frame
    Eigen::Vector3d center;                 // Centroid
    Eigen::Vector3d velocity;               // Average velocity
    Eigen::Matrix3d position_covariance;    // Position uncertainty
    int point_count;                        // Number of merged points
    double quality_score;                   // 0.0 ~ 1.0
    std::vector<int> point_indices;         // Original point IDs
};

struct ClusteringOutput {
    std::vector<ClusteredObject> clusters;
    int pre_count;   // Input detection count
    int post_count;  // Output cluster count
    int merged;      // pre_count - post_count
    double processing_time_ms;
};
```

#### 3.3 Algorithm: Optimized DBSCAN

##### 3.3.1 DBSCAN Overview

**Definition:**
- **ε (epsilon)**: Neighborhood radius (0.8m typical)
- **MinPts**: Minimum points to form cluster (1 typical)
- **Cluster**: Points density-connected within radius

**Algorithm (Pseudocode):**
```pseudocode
DBSCAN(points, ε, MinPts):
    labels ← [-1] * len(points)  // -1 = noise
    cluster_id ← 0
    
    for each point p in points:
        if labels[p] ≠ -1:
            continue  // Already processed
        
        neighbors ← RegionQuery(p, ε)
        
        if len(neighbors) < MinPts:
            labels[p] ← 0  // Mark as noise
            continue
        
        // Start new cluster
        cluster_id ← cluster_id + 1
        ExpandCluster(p, neighbors, cluster_id, ε, MinPts)
    
    return labels

ExpandCluster(point_p, neighbors, cluster_id, ε, MinPts):
    labels[point_p] ← cluster_id
    seed_set ← neighbors
    
    while len(seed_set) > 0:
        current ← seed_set.pop_front()
        
        if labels[current] = -1:
            labels[current] ← cluster_id
        
        if labels[current] ≠ 0:
            continue  // Already assigned
        
        labels[current] ← cluster_id
        neighbor_list ← RegionQuery(current, ε)
        
        if len(neighbor_list) ≥ MinPts:
            seed_set.extend(neighbor_list)
```

**Complexity Analysis:**
- Naive (all-pairs): O(n²) - too slow
- KD-Tree (spatial): O(n log n) - acceptable

##### 3.3.2 KD-Tree Acceleration

**KD-Tree Benefits:**
```
Naive distance calc:  O(n²)  = O(10000) = 10K ops
KD-Tree query:        O(log n) per point ≈ O(13) ops
Total:                O(n log n) vs O(n²)  →  ~1000x speedup
```

**Implementation (using nanoflann):**
```cpp
#include "nanoflann.hpp"

class PointCloud {
public:
    std::vector<Eigen::Vector3d> pts;
    
    size_t kdtree_get_point_count() const { return pts.size(); }
    double kdtree_get_pt(const size_t idx, const size_t dim) const {
        return pts[idx](dim);
    }
};

typedef nanoflann::KDTreeEigenMatrixAdaptor<PointCloud> KDTree_t;

class FastDBSCAN {
    std::unique_ptr<KDTree_t> kdtree_;
    
public:
    std::vector<int> Cluster(const std::vector<Eigen::Vector3d>& points,
                             double epsilon, int min_pts) {
        // Build KD-tree
        PointCloud cloud;
        cloud.pts = points;
        kdtree_ = std::make_unique<KDTree_t>(3, cloud);
        kdtree_->index->buildIndex();
        
        // Run DBSCAN
        std::vector<int> labels(points.size(), -1);
        int cluster_id = 0;
        
        for (size_t i = 0; i < points.size(); i++) {
            if (labels[i] != -1) continue;
            
            // Find neighbors
            std::vector<std::pair<size_t, double>> neighbors;
            kdtree_->index->radiusSearch(
                points[i].data(), epsilon * epsilon, neighbors);
            
            if ((int)neighbors.size() < min_pts) {
                labels[i] = 0;  // Noise
                continue;
            }
            
            // Expand cluster
            cluster_id++;
            ExpandCluster(i, neighbors, labels, cluster_id, 
                         epsilon, min_pts);
        }
        
        return labels;
    }
};
```

**Parameter Selection:**
```
ε (epsilon):
  - mmWave range resolution: ~0.04m
  - Target clustering: 0.6 ~ 1.0m radius
  - Recommendation: ε = 0.8m (good balance)

MinPts:
  - Minimum cluster size: 1 (even single outliers)
  - With small ε, noise/single points naturally separate
  - Recommendation: MinPts = 1
```

#### 3.4 Cluster Statistics Calculation

**For each cluster:**
```cpp
struct ClusterStats {
    // 1. Center position
    Eigen::Vector3d centroid;
    for (auto idx : cluster_indices) {
        centroid += detections[idx].position;
    }
    centroid /= cluster_indices.size();
    
    // 2. Average velocity
    Eigen::Vector3d avg_velocity = Eigen::Vector3d::Zero();
    for (auto idx : cluster_indices) {
        if (detections[idx].has_velocity) {
            avg_velocity += detections[idx].velocity;
        }
    }
    avg_velocity /= valid_velocity_count;
    
    // 3. Position covariance
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (auto idx : cluster_indices) {
        Eigen::Vector3d diff = detections[idx].position - centroid;
        cov += diff * diff.transpose();
    }
    cov /= cluster_indices.size();
    
    // 4. Quality score
    // Example: quality = SNR_avg + log(cluster_size)
    double quality = avg_snr + std::log(cluster_indices.size() + 1);
    quality = std::clamp(quality, 0.0, 1.0);
};
```

#### 3.5 Implementation Checklist

- [ ] **nanoflann** integration for KD-Tree
- [ ] **DBSCAN Core** algorithm
- [ ] **Region Query** with kd_tree
- [ ] **Cluster Expansion**
- [ ] **Statistics Calculation** (center, velocity, covariance)
- [ ] **Quality Scoring** formula
- [ ] **Performance Tests** (n=100, ε=0.8)
- [ ] **Debug Visualization** (color-coded clusters)

---

### Module 3: MULTI-OBJECT TRACKING (MOT)

#### 4.1 Purpose
Maintain persistent object tracks across frames using Kalman filtering and data association.

#### 4.2 Input/Output Specification

**Input:**
```cpp
std::vector<ClusteredObject> measurements;  // ~10-30 objects
```

**Output:**
```cpp
struct ConfirmedTrack {
    int track_id;                           // Persistent ID
    Eigen::VectorXd state;                  // [x, y, z, vx, vy, vz, ...]^T (9D)
    Eigen::MatrixXd P;                      // State covariance (9×9)
    Eigen::Vector3d predicted_position;     // For next frame
    int age;                                // Frames since creation
    int consecutive_misses;                 // Frames without measurement
    int consecutive_hits;                   // Frames with measurement
    uint8_t state_enum;                     // TENTATIVE, CONFIRMED, ABANDONED
    double confidence;                      // 0.0 ~ 1.0
};

struct TrackingOutput {
    std::vector<ConfirmedTrack> confirmed_tracks;
    std::vector<ConfirmedTrack> tentative_tracks;
    int total_active;
    double processing_time_ms;
};
```

#### 4.3 Kalman Filter (Self-Implemented)

##### 4.3.1 State Model

**State Vector (9D):**
```
X(t) = [x, y, z, vx, vy, vz, ax, ay, az]^T

where:
  (x, y, z):       Position (m)
  (vx, vy, vz):    Velocity (m/s)
  (ax, ay, az):    Acceleration (m/s²)
```

**State Transition Model:**
```
X(t+Δt) = F(Δt) · X(t) + w(t)

State Transition Matrix F (9×9) for time step Δt:
F(Δt) = 
⎡ 1  0  0  Δt  0   0  Δt²/2  0      0    ⎤
⎢ 0  1  0  0   Δt  0   0    Δt²/2  0    ⎢
⎢ 0  0  1  0   0  Δt   0     0    Δt²/2⎢
⎢ 0  0  0  1   0   0  Δt     0      0    ⎢
⎢ 0  0  0  0   1   0   0    Δt      0    ⎢
⎢ 0  0  0  0   0   1   0     0     Δt    ⎢
⎢ 0  0  0  0   0   0   1     0      0    ⎢
⎢ 0  0  0  0   0   0   0     1      0    ⎢
⎣ 0  0  0  0   0   0   0     0      1    ⎦

Process Noise w(t) ~ N(0, Q)
```

**Measurement Model:**
```
Z(t) = H · X(t) + v(t)

Measurement Matrix H (3×9) - only position measured:
H = 
⎡ 1  0  0  0  0  0  0  0  0 ⎤
⎢ 0  1  0  0  0  0  0  0  0 ⎢
⎣ 0  0  1  0  0  0  0  0  0 ⎦

Measurement Noise v(t) ~ N(0, R)
```

##### 4.3.2 Kalman Filter Equations

**Prediction Phase:**
```cpp
// 1. Predict state
X_pred = F * X_prev;

// 2. Predict covariance
P_pred = F * P_prev * F.transpose() + Q;
```

**Update Phase (when measurement arrives):**
```cpp
// 1. Measurement innovation (residual)
Y = Z - H * X_pred;  // 3D vector

// 2. Innovation covariance
S = H * P_pred * H.transpose() + R;  // 3×3 matrix

// 3. Kalman gain
K = P_pred * H.transpose() * S.inverse();  // 9×3 matrix

// 4. Update state
X_updated = X_pred + K * Y;

// 5. Update covariance
P_updated = (I - K * H) * P_pred;
```

**C++ Implementation:**
```cpp
class KalmanFilter {
private:
    Eigen::VectorXd X_;      // State (9D)
    Eigen::MatrixXd P_;      // Covariance (9×9)
    Eigen::MatrixXd F_;      // State transition
    Eigen::MatrixXd H_;      // Measurement matrix
    Eigen::MatrixXd Q_;      // Process noise
    Eigen::MatrixXd R_;      // Measurement noise
    
public:
    KalmanFilter()
        : X_(Eigen::VectorXd::Zero(9)),
          P_(Eigen::MatrixXd::Identity(9, 9)),
          F_(Eigen::MatrixXd::Identity(9, 9)),
          H_(Eigen::MatrixXd::Zero(3, 9)),
          Q_(Eigen::MatrixXd::Zero(9, 9)),
          R_(Eigen::MatrixXd::Zero(3, 3)) {
        
        // Setup H: only position measured
        H_(0, 0) = 1; H_(1, 1) = 1; H_(2, 2) = 1;
        
        // Setup Q: process noise
        Q_.diagonal() << 0.01, 0.01, 0.01,    // Position: σ²_x
                        0.25, 0.25, 0.25,    // Velocity: σ²_v
                        1.0, 1.0, 1.0;       // Acceleration: σ²_a
        
        // Setup R: measurement noise (sensor spec)
        R_.diagonal() << 0.04, 0.04, 0.04;   // Position: (0.2m)²
    }
    
    void Predict(double dt) {
        UpdateStateTransition(dt);
        X_ = F_ * X_;
        P_ = F_ * P_ * F_.transpose() + Q_;
    }
    
    void Update(const Eigen::Vector3d& measurement) {
        // Innovation
        Eigen::Vector3d y = measurement - H_ * X_;
        
        // Innovation covariance
        Eigen::Matrix3d S = H_ * P_ * H_.transpose() + R_;
        
        // Kalman gain
        Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
        
        // Update state
        X_ = X_ + K * y;
        
        // Update covariance
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(9, 9);
        P_ = (I - K * H_) * P_;
    }
    
    double MahalanobisDistance(const Eigen::Vector3d& meas) const {
        Eigen::Vector3d innov = meas - H_ * X_;
        Eigen::Matrix3d S = H_ * P_ * H_.transpose() + R_;
        return innov.transpose() * S.inverse() * innov;
    }
    
private:
    void UpdateStateTransition(double dt) {
        // Update F matrix for current dt
        F_(0, 3) = dt; F_(0, 6) = dt*dt/2;
        F_(1, 4) = dt; F_(1, 7) = dt*dt/2;
        F_(2, 5) = dt; F_(2, 8) = dt*dt/2;
        F_(3, 6) = dt;
        F_(4, 7) = dt;
        F_(5, 8) = dt;
    }
};
```

**Covariance Matrix Values (Tuning):**
```
Q (Process Noise):
  Position: σ² = 0.01 (assume 0.1m/s² acceleration jitter)
  Velocity: σ² = 0.25 (assume 0.5m/s velocity jitter)
  Acceleration: σ² = 1.0 (assume 1.0m/s² acceleration jitter)

R (Measurement Noise):
  Position: σ² = 0.04 (assume 0.2m measurement std from sensor spec)

P_init (Initial Uncertainty):
  Large initial uncertainty: diag(I)*10 (gradually converges)
```

##### 4.3.3 Mahalanobis Distance

**Purpose:** Normalized distance accounting for covariance

**Formula:**
```
d²_M = (z - x̂)ᵀ S⁻¹ (z - x̂)

where:
  z:  Measurement
  x̂:  Prediction
  S:  Innovation covariance = H*P*H^T + R

Interpretation:
  d²_M ~ χ² distribution (DOF=3)
  P(d²_M < 5.99) = 95%  ← Standard gate threshold
```

**Usage:**
```cpp
if (kf.MahalanobisDistance(measurement) < 5.99) {
    // Valid measurement for this track
    kf.Update(measurement);
} else {
    // Measurement too far - don't update
    // Track will only use prediction
}
```

#### 4.4 Data Association (Track-to-Measurement Assignment)

##### 4.4.1 Hungarian Algorithm

**Problem:** Find optimal matching between tracks and measurements

**Cost Matrix:**
```cpp
// Cost[i][j] = cost of assigning track_i to measurement_j
Eigen::MatrixXd cost(num_tracks, num_measurements);

for (int i = 0; i < num_tracks; i++) {
    for (int j = 0; j < num_measurements; j++) {
        double dist = tracks[i].kf.MahalanobisDistance(
            measurements[j].position);
        
        if (dist < GATING_THRESHOLD) {  // ~5.99
            cost(i, j) = dist;
        } else {
            cost(i, j) = INF;  // Incompatible
        }
    }
}
```

**Hungarian Algorithm (Complexity: O(n³)):**
```pseudocode
Hungarian(cost_matrix):
    // Reduction phase
    for each row:
        subtract min(row) from all elements in row
    
    for each column:
        subtract min(column) from all elements in column
    
    // Covering phase
    while uncovered_zeros < num_rows:
        // Cover zeros with minimum lines
        lines = FindMinimumCover()
        
        if lines < num_rows:
            // Update matrix
            min_uncov = min(uncovered elements)
            UpdateMatrix(min_uncov)
    
    return optimal_assignment
```

**C++ Options:**
```cpp
// Option 1: Simple implementation (educational)
class SimpleHungarian {
public:
    std::vector<int> Solve(const Eigen::MatrixXd& cost_matrix);
};

// Option 2: Use open-source library
#include "hungarian.h"  // Munkres algorithm

// Option 3: Approximate O(n²) for small n
// Since n_tracks ≤ 20, O(n³) is acceptable
```

##### 4.4.2 JPDA (Joint Probabilistic Data Association) - Optional

**Simplified JPDA Concept:**
```
Instead of hard assignment (track → single measurement),
allow track to consider weighted sum of nearby measurements.

Weight calculation:
  β_ij = likelihood(measurement_j | track_i) / 
         Σ_k likelihood(measurement_k | track_i)

  Update with weighted measurement:
  z_i^weighted = Σ_j β_ij * measurement_j
```

**When to use:**
- Very crowded scenes (multiple objects nearby)
- Ambiguous assignments
- More robust but computationally expensive

**For initial implementation, Hungarian is sufficient.**

#### 4.5 Track State Machine

**States:**
```
TENTATIVE  → CONFIRMED → ABANDONED
   ↑            ↓
   └────────────┘
```

**Transitions:**
```cpp
enum TrackState {
    TENTATIVE   = 0,  // New, not yet confirmed
    CONFIRMED   = 1,  // Active track
    ABANDONED   = 2   // Archived (logged but not output)
};

struct TracklogicParams {
    int confirmation_age = 3;        // Frames to confirm
    int max_consecutive_misses = 5;  // Frames to delete
    int max_age = 300;              // Max frames alive
};

// State transitions
for (auto& track : active_tracks) {
    if (track.state == TENTATIVE) {
        if (track.age > confirmation_age && 
            track.hits / track.age > 0.5) {
            track.state = CONFIRMED;
        }
    }
    
    if (track.consecutive_misses > max_consecutive_misses) {
        track.state = ABANDONED;
        RemoveTrack(track.id);
    }
    
    if (track.age > max_age) {
        track.state = ABANDONED;
        RemoveTrack(track.id);
    }
}
```

#### 4.6 Implementation Checklist

- [ ] **Kalman Filter** class (9×9 state model)
- [ ] **Hungarian Algorithm** or wrapper
- [ ] **Mahalanobis Distance** calculation
- [ ] **Track State Machine** (TENTATIVE/CONFIRMED/ABANDONED)
- [ ] **Track Management** (creation/deletion)
- [ ] **Thread-safe Track List** (concurrent access)
- [ ] **Performance Monitoring**
- [ ] **Integration Tests**

---

## III. C++ PROJECT STRUCTURE & BUILD

### 3.1 Directory Layout

```
RadarTracking/
├── CMakeLists.txt                    # Build configuration
├── .gitignore
├── README.md
├── SPECIFICATION.md                  # This document
│
├── include/
│   ├── radar_tracker.hpp             # Main API
│   ├── data_types.hpp                # Common structures
│   ├── preprocessor.hpp              # Module 1
│   ├── clusterer.hpp                 # Module 2
│   ├── multi_object_tracker.hpp      # Module 3
│   ├── kalman_filter.hpp
│   ├── hungarian_matcher.hpp
│   ├── ring_buffer.hpp               # Lock-free buffer
│   ├── serial_port.hpp
│   └── utils.hpp
│
├── src/
│   ├── main.cpp
│   ├── radar_tracker.cpp
│   ├── preprocessor.cpp
│   ├── clusterer.cpp
│   ├── multi_object_tracker.cpp
│   ├── kalman_filter.cpp
│   ├── hungarian_matcher.cpp
│   ├── ring_buffer.cpp
│   ├── serial_port.cpp
│   └── utils.cpp
│
├── test/
│   ├── CMakeLists.txt
│   ├── test_kalman_filter.cpp
│   ├── test_dbscan.cpp
│   ├── test_hungarian.cpp
│   ├── test_integration.cpp
│   └── fixtures/  # Test data
│
├── config/
│   ├── default_config.yaml
│   ├── jetson_nano.yaml
│   └── sensor_calib.json
│
├── docs/
│   ├── API.md
│   ├── TUNING.md
│   └── TROUBLESHOOTING.md
│
└── data/
    ├── recorded_frames/
    ├── calibration/
    └── benchmark_results/
```

### 3.2 Dependencies

```cmake
# Essential
- Eigen 3.4+         # Linear algebra
- C++17 compiler

# Recommended
- nanoflann          # KD-Tree (spatial indexing)
- fmt                # Logging
- Boost              # Thread utilities (optional)
- OpenCV 4.5+        # Visualization (optional)

# Testing
- GoogleTest (GTest) # Unit testing
- csv-parser         # Data loading

# Build
- CMake 3.16+
- Ninja (faster than Make)
```

### 3.3 Sample CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.16)
project(RadarTracking CXX)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Optimization
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -DNDEBUG")
    message(STATUS "Build type: Release (optimized)")
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g -Wall -Wextra")
    message(STATUS "Build type: Debug (with symbols)")
endif()

# FindPackage
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(fmt REQUIRED)

# Optional: OpenCV for visualization
find_package(OpenCV 4.0 QUIET COMPONENTS core visualization)
if(OpenCV_FOUND)
    message(STATUS "OpenCV found: visualization enabled")
else()
    message(STATUS "OpenCV not found: visualization disabled")
endif()

# Library: Core tracking engine
add_library(radar_tracking SHARED
    src/preprocessor.cpp
    src/clusterer.cpp
    src/kalman_filter.cpp
    src/multi_object_tracker.cpp
    src/hungarian_matcher.cpp
    src/radar_tracker.cpp
)

target_include_directories(radar_tracking PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(radar_tracking PUBLIC
    Eigen3::Eigen
    fmt::fmt
    ${OpenCV_LIBRARIES}
)

# Executable: Main application
add_executable(radar_tracking_app src/main.cpp)
target_link_libraries(radar_tracking_app PRIVATE radar_tracking)

# Tests
enable_testing()
add_subdirectory(test)

# Installation
install(TARGETS radar_tracking radar_tracking_app
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
```

### 3.4 Core Classes (Header Sketches)

**include/data_types.hpp:**
```cpp
#pragma once
#include <Eigen/Dense>
#include <vector>
#include <cstdint>

// Preprocessor output
struct FilteredDetection {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    double snr;
    double confidence;
    Eigen::Matrix3d position_covariance;
};

// Clusterer output
struct ClusteredObject {
    int cluster_id;
    Eigen::Vector3d center;
    Eigen::Vector3d velocity;
    Eigen::Matrix3d position_covariance;
    int point_count;
    double quality_score;
};

// Track output
struct ConfirmedTrack {
    int track_id;
    Eigen::Vector3d position;        // Filter state: x
    Eigen::Vector3d velocity;        // Filter state: vx
    Eigen::Vector3d acceleration;    // Filter state: ax
    double confidence;
    int age;
    int consecutive_misses;
    uint8_t state;  // TENTATIVE, CONFIRMED, ABANDONED
};
```

**include/radar_tracker.hpp:**
```cpp
#pragma once
#include "data_types.hpp"
#include <memory>

class RadarTracker {
public:
    RadarTracker(const std::string& config_file);
    ~RadarTracker();
    
    bool Initialize();
    void Start();
    void Stop();
    
    // Main processing
    std::vector<ConfirmedTrack> ProcessFrame(
        const std::vector<FilteredDetection>& detections,
        double timestamp);
    
    // Performance metrics
    struct Metrics {
        double fps;
        double latency_ms;
        int num_tracks;
        int total_detections;
    };
    Metrics GetMetrics() const;
    
private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
```

---

## IV. REAL-TIME ARCHITECTURE

### 4.1 Latency Budget

```
Total Frame Budget: 33.3ms (30 FPS)

Phase                    Budget      Typical    Margin
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Data Reception           5ms         3ms        40%
Preprocessing Filtering  8ms         7ms        12%
Clustering (DBSCAN)      8ms         5ms        37%
Tracking (KF)            8ms         6ms        25%
Association (Hungarian)  2ms         1ms        50%
Output/Logging           1ms         1ms        0%
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
TOTAL:                   33ms        23ms       30% margin
```

### 4.2 Threading Model

**Architecture:**
```
Thread 1: Reception (Producer)
    ├─ Continuous SerialPort Read
    └─ Push → RingBuffer

Thread 2: Processing (Consumer)
    ├─ Pop ← RingBuffer
    ├─ Preprocessing Filter
    ├─ Clustering (DBSCAN)
    ├─ Tracking (KF + Association)
    └─ Push → OutputQueue

Thread 3: Output (Consumer)
    ├─ Pop ← OutputQueue
    ├─ Logging/Storage
    └─ Visualization (optional)

Synchronization: Lock-free queues (boost::lockfree)
               or Condition variables
```

**Benefits:**
- Independent I/O from computation
- Better cache locality
- Scalable to 4+ threads on Jetson Xavier

### 4.3 Memory Management

**Pre-allocation Strategy:**
```cpp
class MemoryPool {
    static const size_t MAX_DETECTIONS = 1000;
    static const size_t MAX_CLUSTERS = 100;
    static const size_t MAX_TRACKS = 20;
    
    std::vector<FilteredDetection> detection_pool_;
    std::vector<ClusteredObject> cluster_pool_;
    
    // Single allocation at startup, no per-frame allocation
    MemoryPool() {
        detection_pool_.resize(MAX_DETECTIONS);
        cluster_pool_.resize(MAX_CLUSTERS);
    }
    
    // Reset for next frame
    void Reset() {
        detection_count_ = 0;
        cluster_count_ = 0;
    }
};
```

**Benefits:**
- O(1) allocation/deallocation
- Predictable latency (no GC pauses)
- Better cache coherence

### 4.4 Lock-free Data Structures

```cpp
#include <boost/lockfree/queue.hpp>

// ProcessedFrame queue (Thread 1 → Thread 2)
boost::lockfree::queue<ProcessedFrame*> frame_queue(1000);

// OutputTrack queue (Thread 2 → Thread 3)
boost::lockfree::queue<ConfirmedTrack*> track_queue(1000);

// Speed: ~300 ns per queue operation (vs 5000 ns with mutex)
```

### 4.5 Performance Profiling

```cpp
class Timer {
    std::chrono::high_resolution_clock::time_point start_;
    
public:
    void Start() { 
        start_ = std::chrono::high_resolution_clock::now();
    }
    
    double ElapsedMs() {
        auto end = std::chrono::high_resolution_clock::now();
        return std::chrono::duration<double, std::milli>(
            end - start_).count();
    }
};

// Usage
Timer timer;

timer.Start();
auto[filtered] = preprocessor_.Process(raw_detections);
printf("Preprocessing: %.2f ms\n", timer.ElapsedMs());

timer.Start();
auto[clusters] = clusterer_.Cluster(filtered);
printf("Clustering: %.2f ms\n", timer.ElapsedMs());

timer.Start();
auto[tracks] = tracker_.Update(clusters, timestamp);
printf("Tracking: %.2f ms\n", timer.ElapsedMs());
```

---

## V. OPTIMIZATION CHECKLIST

### Algorithm Level
- [ ] DBSCAN with KD-Tree (O(n log n))
- [ ] Hungarian algorithm (exact, O(n³) for n≤20)
- [ ] Mahalanobis distance (caching, SIMD)
- [ ] Matrix inversion (Cholesky decomposition)

### Memory Level
- [ ] Pre-allocated buffers (no per-frame malloc)
- [ ] Object pools for frequent allocations
- [ ] Stack-based small objects (<256 bytes)
- [ ] Cache-friendly data layout

### Parallelization
- [ ] Producer-Consumer threading
- [ ] Lock-free queues (boost::lockfree)
- [ ] SIMD vectorization (Eigen auto)
- [ ] CPU affinity settings

### Platform
- [ ] ARM NEON optimization (Jetson)
- [ ] -O3 -march=native compilation
- [ ] Profile-guided optimization (PGO, optional)

---

## VI. TESTING & VALIDATION

### Unit Tests

```cpp
// test/test_kalman_filter.cpp
#include <gtest/gtest.h>
#include "kalman_filter.hpp"

TEST(KalmanFilterTest, LinearMotion) {
    KalmanFilter kf;
    kf.Initialize({0, 0, 0}, {1, 0, 0});  // pos, vel
    
    // Constant velocity: x = x0 + v*t
    for (int i = 0; i < 100; i++) {
        kf.Predict(0.033);  // 30fps
        kf.Update({i * 0.033, 0, 0});  // Measurement
    }
    
    auto pos = kf.GetPosition();
    EXPECT_NEAR(pos[0], 3.30, 0.10);  // 100 * 0.033
}

// test/test_dbscan.cpp
TEST(DBSCANTest, SimpleClustering) {
    std::vector<Eigen::Vector3d> points = {
        {0, 0, 0}, {0.1, 0, 0}, {0.2, 0, 0},    // Cluster 1
        {5, 5, 0}, {5.1, 5, 0}                 // Cluster 2
    };
    
    Clusterer clusterer;
    auto clusters = clusterer.Cluster(points);
    
    EXPECT_EQ(clusters.size(), 2);
}
```

### Integration Tests

```cpp
// test/test_integration.cpp
TEST(IntegrationTest, RecordedDataset) {
    RadarTracker tracker("config/default_config.yaml");
    ASSERT_TRUE(tracker.Initialize());
    
    // Load frames from recorded data
    auto frames = LoadRecordedFrames("data/recorded_frames");
    
    std::vector<double> latencies;
    for (const auto& frame : frames) {
        auto start = std::chrono::high_resolution_clock::now();
        auto tracks = tracker.ProcessFrame(frame);
        auto end = std::chrono::high_resolution_clock::now();
        
        auto latency_ms = 
            std::chrono::duration<double, std::milli>(end - start).count();
        latencies.push_back(latency_ms);
        
        EXPECT_LT(latency_ms, 50);  // <50ms per frame
    }
    
    // Statistics
    double avg_latency = 
        std::accumulate(latencies.begin(), latencies.end(), 0.0) /
        latencies.size();
    double fps = 1000.0 / avg_latency;
    
    std::cout << "Average latency: " << avg_latency << " ms\n";
    std::cout << "Effective FPS: " << fps << "\n";
    
    EXPECT_GT(fps, 25);  // Minimum 25 FPS
}
```

### Real-World Test Scenarios

1. **Highway Scenario**
   - Multiple vehicles at various speeds
   - Relative velocity: -50 to +50 m/s
   - Expected: Stable tracking, <5% ID switches

2. **Urban Scenario**
   - Close-by pedestrians/cyclists
   - Speed range: 0-30 m/s
   - Expected: Good clustering, <10% false positives

3. **Parking Scenario**
   - Stationary vehicles (should be filtered)
   - Moving vehicles (slow)
   - Expected: No tracking of static objects

4. **Edge Cases**
   - Sudden acceleration/deceleration
   - Lane changes
   - Fast left/right turns
   - Expected: Graceful degradation, no crashes

---

## VII. TROUBLESHOOTING

### Low FPS
1. Check CPU usage (should be <50%)
2. Verify thread affinity (CPU pinning)
3. Profile each module (use timers)
4. Reduce cluster count with stricter DBSCAN parameters

### High Latency Spikes
1. Check for memory allocations (use -fno-std-simd-warn)
2. Verify no page faults (use perf record)
3. Check thermal throttling
4. Reduce clustering epsilon

### Track Loss
1. Verify sensor mounting/calibration
2. Check Kalman Filter Q/R parameters
3. Tune confirmation thresholds
4. Check gating distance (Mahalanobis < 5.99)

---

## VIII. MIGRATION ROADMAP: MATLAB → C++

### Phase 1: Setup (Week 1)
- [ ] Project structure & CMake
- [ ] Dependency setup (Eigen, nanoflann)
- [ ] Build pipeline

### Phase 2: Preprocessing (Week 1-2)
- [ ] Serial port communication
- [ ] Raw data parsing
- [ ] Filtering pipeline
- [ ] Unit tests

### Phase 3: Clustering (Week 2)
- [ ] KD-Tree integration
- [ ] DBSCAN algorithm
- [ ] Cluster statistics

### Phase 4: Tracking (Week 3-4)
- [ ] Kalman Filter
- [ ] Hungarian matcher
- [ ] Track management
- [ ] Integration tests

### Phase 5: Optimization (Week 4-5)
- [ ] Profiling & tuning
- [ ] Memory optimization
- [ ] Multi-threading
- [ ] Performance validation

### Phase 6: Deployment (Week 5-6)
- [ ] Jetson Nano porting
- [ ] Documentation
- [ ] Demo & presentation

**Total Duration: 4-6 weeks**

---

## APPENDIX: References

### Algorithms
- Kalman Filter: Welch & Bishop, "An Introduction to the Kalman Filter" (2006)
- Hungarian Algorithm: Munkres, "Algorithms for Assignment and Transportation Problems" (1957)
- DBSCAN: Ester et al., "A Density-Based Algorithm for Discovering Clusters" (1996)
- JPDA: Bar-Shalom & Fortmann, "Tracking and Data Association" (1988)

### Libraries
- Eigen: http://eigen.tuxfamily.org/
- nanoflann: https://github.com/jlblancoc/nanoflann
- Boost (lock-free): https://www.boost.org/
- GTest: https://github.com/google/googletest

### Tools
- CMake: https://cmake.org/
- Valgrind: http://valgrind.org/ (memory profiling)
- perf: https://linux-audit.com/perf-the-linux-profiling-tool/

### Hardware
- Jetson Nano Datasheet: https://developer.nvidia.com/jetson-nano
- IWR6843ISK: https://www.ti.com/tool/IWR6843ISK

---

**Document Version:** 1.0  
**Last Updated:** 2025-02-25  
**Status:** Complete Industry-Standard Specification  
**Target Audience:** ADAS Development Team, Capstone Project
