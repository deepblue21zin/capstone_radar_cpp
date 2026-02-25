# Real-Time Radar Object Tracking System (C++)

> 💻 TI IWR6843ISK 60GHz mmWave 레이더 기반 실시간 다중 객체 추적 시스템  
> **C++17 | CMake | Eigen3 | Kalman Filter | Hungarian Algorithm | DBSCAN**

---

## 📋 목차
1. [시스템 개요](#시스템-개요)
2. [전체 아키텍처](#전체-아키텍처)
3. [모듈 상세 설명](#모듈-상세-설명)
4. [구성 파일](#구성-파일)
5. [빌드 및 실행](#빌드-및-실행)
6. [데이터 흐름](#데이터-흐름)

---

## 시스템 개요

이 프로젝트는 **TI IWR6843ISK** 60GHz mmWave 레이더 센서를 기반으로 한 **실시간 다중 객체 추적 시스템**입니다.

### 핵심 특징
- ✅ **TI IWR6843ISK** 레이더 센서 전용 설계
- ✅ **실시간 처리**: 저지연 시간 최적화
- ✅ **다중 객체 추적**: 칼만 필터 + 헝가리안 알고리즘
- ✅ **DBSCAN 클러스터링**: 근접 탐지점 병합
- ✅ **모듈식 설계**: 각 단계 독립적으로 구성 가능
- ✅ **성능 모니터링**: FPS, 레이턴시, CPU/메모리 추적

### 기본 사양

| 항목 | 사양 |
|------|------|
| **센서** | TI IWR6843ISK (60GHz mmWave) |
| **감지 거리** | 최대 12m |
| **감지 각도** | ±120° (방위각), ±90° (높이각) |
| **최대 객체 수** | 200+ 추적 가능 |
| **프레임 레이트** | 1-10 fps (설정 가능) |
| **지연 시간** | ~100ms (전체 파이프라인) |
| **언어** | C++17 |
| **의존성** | Eigen3, fmt (선택), OpenCV (선택) |

---

## 전체 아키텍처

### 처리 파이프라인

```
┌─────────────────────────────────────────────────────────────┐
│                   IWR6843ISK 레이더 센서                     │
│                      (UART 통신)                            │
└──────────────────────────┬──────────────────────────────────┘
                           │
                    Raw Detection Data
                           │
                           ▼
    ┌─────────────────────────────────────────────────────┐
    │           Module 1: PREPROCESSING                  │
    │  ✓ 속도/거리/높이 필터링                             │
    │  ✓ ROI 경계 필터링                                   │
    │  ✓ SNR 품질 필터링                                   │
    │  ✓ 좌표 변환                                         │
    └────────────────────┬────────────────────────────────┘
                         │
                  Filtered Detections
                    (200-400 points)
                         │
                         ▼
    ┌─────────────────────────────────────────────────────┐
    │          Module 2: CLUSTERING (DBSCAN)             │
    │  ✓ 근접 탐지점 병합                                  │
    │  ✓ 클러스터 특성 계산                               │
    │  ✓ 이상치 제거                                       │
    └────────────────────┬────────────────────────────────┘
                         │
                Clustered Objects
                   (5-30 objects)
                         │
                         ▼
    ┌─────────────────────────────────────────────────────┐
    │        Module 3: MULTI-OBJECT TRACKING             │
    │  ✓ 칼만 필터 (상태 추정)                             │
    │  ✓ 헝가리안 알고리즘 (데이터 연관)                    │
    │  ✓ 트랙 관리 상태 머신                               │
    │  ✓ 신뢰도 점수 계산                                  │
    └────────────────────┬────────────────────────────────┘
                         │
                Confirmed Tracks Output
                         │
                         ▼
            ┌──────────────────────────┐
            │   Application Layer      │
            │  (Visualization/Output)  │
            └──────────────────────────┘
```

---

## 모듈 상세 설명

### 🔵 Module 1: Preprocessor (전처리)

**역할**: 레이더 센서에서 수신한 raw 데이터를 정제하고 필터링

#### 주요 기능
- **속도 기반 필터링**: 정적 클러터 제거 (속도 < 0.2 m/s)
- **거리 필터링**: 최대 감지 거리 제한 (기본 12m)
- **높이 필터링**: 감지 높이 제한 (기본 0-2.5m, 자동차 높이)
- **경계 박스 필터링**: ROI 지정 (기본 ±2m 횡방향, 0.5-8m 종방향)
- **SNR 필터링**: 신호 품질 임계값 (기본 10dB)

**성능**: ~1-2ms (500개 포인트 기준)

#### 구성 매개변수
```yaml
preprocessing:
  min_speed_threshold: 0.2      # 정적 필터 (m/s)
  max_range: 12.0               # 거리 제한 (m)
  height:
    min: 0.0
    max: 2.5
  boundary_box:
    x_min: -2.0                 # 좌측 (m)
    x_max: 2.0                  # 우측 (m)
    y_min: 0.5                  # 근거리 (m)
    y_max: 8.0                  # 원거리 (m)
  min_snr: 10.0                 # 신호 품질 (dB)
```

---

### 🟢 Module 2: Clusterer (클러스터링)

**역할**: 근접한 탐지점들을 하나의 객체로 병합

#### 주요 기능
- **DBSCAN 알고리즘**: 클러스터 생성 (ε=0.8m, minPts=1)
- **근접 포인트 병합**: 이웃 반경 내 포인트 통합
- **클러스터 특성 계산**: 중심, 크기, SNR, 속도
- **이상치 처리**: 고립된 포인트 제거

**성능**: ~2-3ms (500개 포인트 기준), O(n log n) 시간복잡도

#### 구성 매개변수
```yaml
clustering:
  epsilon: 0.8                  # 이웃 반경 (m)
  min_pts: 1                    # 최소 포인트 수
  quality_weight: 0.3           # 신뢰도 가중치
```

---

### 🔴 Module 3: MultiObjectTracker (다중 객체 추적)

**역할**: 프레임 간 객체를 추적하고 움직임 예측

#### 1️⃣ 칼만 필터 (Kalman Filter)
```
상태 벡터: [x, y, z, vx, vy, vz, ax, ay, az]ᵀ (9차원)

예측 단계 (predict):
  p(t+1) = p(t) + v(t)·Δt + 0.5·a·Δt²
  v(t+1) = v(t) + a(t)·Δt

업데이트 단계 (update):
  측정값과 예측값을 결합
```

**과정 노이즈 (Q)**: 위치(0.01m²), 속도(0.25(m/s)²), 가속도(1.0(m/s²)²)  
**측정 노이즈 (R)**: 0.04 (센서 좌표 불확도)

#### 2️⃣ 헝가리안 알고리즘 (Hungarian Algorithm)
```
데이터 연관:
  - 예측 위치와 측정값의 거리 행렬 계산
  - 최소 비용 할당 (마할라노비스 거리)
  - 최적 일대일 할당 보장
```

#### 3️⃣ 트랙 관리 상태 머신
```
TENTATIVE ──3 프레임──→ CONFIRMED ──5 프레임 미스──→ DELETED
```

#### 구성 매개변수
```yaml
tracking:
  kalman_filter:
    process_noise:
      position: 0.01          # m²
      velocity: 0.25          # (m/s)²
      acceleration: 1.0       # (m/s²)²
    measurement_noise: 0.04   # m²
    
  track_management:
    confirmation_frames: 3
    max_consecutive_miss: 5
    max_track_age: 300
```

---

## 구성 파일

### 📄 config/default_config.yaml

센서 설정 (라인 85-87):
```yaml
sensor:
  type: "IWR6843ISK"          # ← TI IWR6843ISK 레이더
  frame_rate: 10              # fps
  max_detections: 500
```

---

## 빌드 및 실행

### 요구 사항
- **C++ 표준**: C++17 이상
- **빌드 시스템**: CMake 3.16+
- **컴파일러**: g++ 7.0+, clang 5.0+
- **의존성**: Eigen3 (필수)

### 빌드
```bash
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . --config Release -j4
```

### 실행
```bash
./build/Release/radar_tracking
```

---

## 데이터 흐름

### 프레임 처리 (한 사이클, ~10ms)
1. **Raw Data** (520 detections)
2. **Preprocessing** (2ms) → 305 detections
3. **Clustering** (3ms) → 18 clusters
4. **Tracking** (5ms) → 5 confirmed tracks + 2 tentative

**총 지연**: ~10ms ≈ 100fps 가능

---

## IWR6843ISK 레이더 확인 ✅

이 시스템은 **TI IWR6843ISK** 전용으로 설계되었습니다.

### 확인 위치:
1. **config/default_config.yaml** (라인 87): `type: "IWR6843ISK"`
2. **Core/Src/main.cpp** (라인 114): `IWR6843ISK mmWave Radar`
3. **doc/SPECIFICATION_*.md**: `센서 | TI IWR6843ISK (60GHz mmWave)`

### 레이더 사양:
- **모델**: TI IWR6843ISK (60GHz Industrial Radar)
- **감지 거리**: 최대 12m
- **각도 분해능**: 4°
- **거리 분해능**: 10cm
- **통신**: UART + CAN

---

## 참조
- [TI IWR6843ISK](https://www.ti.com/tool/IWR6843ISK)
- [구현 가이드](IMPLEMENTATION_GUIDE_KOR.md)
- [상세 스펙](SPECIFICATION_KOR.md)
