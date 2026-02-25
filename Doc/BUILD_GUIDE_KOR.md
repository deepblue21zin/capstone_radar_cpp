# C++ 빌드 및 실행 가이드

## 📋 사전 조건

### 필수 설치
```bash
# Windows
1. Visual Studio 2019 이상 (C++17 지원)
   또는 MinGW (GCC 9+)

2. CMake 3.16+
   https://cmake.org/download/

3. Eigen 3.3+ (헤더만)
   https://eigen.tuxfamily.org/
   → 프로젝트 폴더 내 third_party/eigen 에 압축해제

4. Git (선택사항)
   https://git-scm.com/
```

---

## 🏗️ 빌드 절차

### **Windows (Visual Studio)**

#### 1단계: 종속성 준비
```cmd
# RadarTracking_CPP 디렉토리 생성
mkdir RadarTracking_CPP
cd RadarTracking_CPP

# Eigen 다운로드 (3.4.0)
# https://eigen.tuxfamily.org/ → 다운로드
# 압축 해제:
mkdir third_party
# eigen-3.4.0 폴더를 third_party/eigen 로 복사
```

#### 2단계: 빌드
```cmd
# 빌드 디렉토리 생성
mkdir build
cd build

# CMake 생성 (Release 최적화)
cmake -DCMAKE_BUILD_TYPE=Release ..

# 또는 Visual Studio IDE에서 빌드
cmake --build . --config Release --parallel 4

# 빌드 완료
# → build/Release/radar_tracking_main.exe 생성
```

#### 3단계: 실행
```cmd
# 현재 디렉토리: RadarTracking_CPP/build

# 기본 실행 (더미 데이터)
./Release/radar_tracking_main.exe

# 커스텀 설정 사용
./Release/radar_tracking_main.exe ../config/jetson_nano.yaml
```

---

### **Linux (Ubuntu/Jetson)**

#### 1단계: 종속성 설치
```bash
# 업데이트
sudo apt update
sudo apt upgrade -y

# 빌드 도구
sudo apt install -y build-essential cmake git

# Eigen (optional, header-only이므로 필수 아님)
# 수동 다운로드 후 third_party/에 위치

# (선택) OpenCV 설치 (시각화)
sudo apt install -y libopencv-dev

# (선택) Google Test 설치 (단위 테스트)
sudo apt install -y google-mock
```

#### 2단계: 빌드
```bash
# 빌드 디렉토리 생성
mkdir build
cd build

# CMake 생성 (Release)
cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_FLAGS="-O3 -march=native" ..

# 병렬 빌드 (프로세서 수만큼)
cmake --build . -j$(nproc)

# 빌드 완료
# → build/radar_tracking_main 생성
```

#### 3단계: 실행
```bash
./radar_tracking_main

# 또는 with profiling
time ./radar_tracking_main

# Jetson Nano 최적화 실행
./radar_tracking_main ../config/jetson_nano.yaml
```

---

### **Jetson Nano 최적화**

```bash
# 추가 최적화 플래그
cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_FLAGS="-O3 -march=armv8-a+simd -ftree-vectorize" ..

# 성능 모드 (냉각 필요)
sudo jetson_clocks

# 빌드
cmake --build . -j4
```

---

## 📊 성능 검증

### **FPS 측정**
```bash
# 실행 후 마지막 성능 지표 확인
# 예상: 30+ FPS, <33ms 레이턴시

# 상세 성능 분석
./radar_tracking_main 2>&1 | grep -E "FPS:|Latency:"
```

### **메모리 프로파일링** (Linux)
```bash
# Valgrind로 메모리 누수 체크
valgrind --leak-check=full \
         --show-leak-kinds=all \
         ./radar_tracking_main
```

### **CPU 프로파일링** (Linux)
```bash
# perf로 CPU 사용률 분석
sudo perf record ./radar_tracking_main
sudo perf report
```

---

## 🔧 디버깅

### **Debug 빌드**
```bash
cmake -DCMAKE_BUILD_TYPE=Debug ..
cmake --build .

# GDB 디버거
gdb ./radar_tracking_main
(gdb) run
(gdb) bt              # Stack trace
(gdb) quit            # 종료
```

### **빌드 문제 해결**

#### 문제 1: Eigen 찾을 수 없음
```
Error: Could not find Eigen3

해결:
1. Eigen을 third_party/eigen 에 수동으로 복사
2. CMakeLists.txt에서 find_package(Eigen3...) 주석 처리
   → include_directories(${CMAKE_SOURCE_DIR}/third_party)
```

#### 문제 2: C++17 미지원
```
Error: -std=c++17 not supported

해결:
- GCC 7+ 또는 Clang 5+ 필요
- 또는 set(CMAKE_CXX_STANDARD 14) 로 다운그레이드
```

#### 문제 3: 충돌하는 라이브러리
```
해결:
cmake --fresh    # 캐시 초기화
rm -rf build && mkdir build
```

---

## 📁 파일 구조

```
RadarTracking_CPP/
├── CMakeLists.txt              # 빌드 설정
├── include/
│   ├── radar_tracker.hpp       ✅ 완성
│   ├── data_types.hpp          ✅ 완성
│   ├── kalman_filter.hpp       ✅ 완성
│   ├── clusterer.hpp           ✅ 완성
│   ├── hungarian_matcher.hpp   ✅ 완성
│   ├── preprocessor.hpp        ✅ 완성
│   └── multi_object_tracker.hpp ✅ 완성
├── src/
│   ├── main.cpp                ✅ 실행 가능
│   ├── kalman_filter.cpp       ✅ 완성
│   ├── preprocessor.cpp        ✅ 기본 구현
│   ├── clusterer.cpp           ✅ DBSCAN 구현
│   ├── multi_object_tracker.cpp ✅ 트래킹 구현
│   ├── hungarian_matcher.cpp   ✅ 매칭 구현
│   └── radar_tracker.cpp       ✅ 통합 구현
├── config/
│   └── default_config.yaml     ✅ 설정
├── third_party/
│   └── eigen/                  ← 필수 (수동 설치)
└── build/                      ← CMake 생성 (자동)
    └── Release/
        └── radar_tracking_main ← 실행 파일
```

---

## ⚡ 빠른 시작 (30초)

### **Windows**
```cmd
mkdir RadarTracking_CPP && cd RadarTracking_CPP

# Eigen 설정 (생략 가능)
mkdir third_party

# 제공된 파일 복사
# include/, src/, config/, CMakeLists.txt 복사

# 빌드 및 실행
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . --config Release
./Release/radar_tracking_main.exe
```

### **Linux**
```bash
mkdir RadarTracking_CPP && cd RadarTracking_CPP
# 파일 복사...
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
./radar_tracking_main
```

---

## 📈 예상 결과

```
╔════════════════════════════════════════════════════════════╗
║  RadarTracker System Initialization                       ║
╚════════════════════════════════════════════════════════════╝

🔧 [Module 1] Preprocessing initialization...
✅ [Module 1] Initialized
   - Speed threshold: 0.20 m/s
   - Max range: 12.0 m
   ...

🔧 [Module 2] Clustering initialization...
✅ [Module 2] Initialized
   - DBSCAN epsilon: 0.80 m
   ...

🔧 [Module 3] Tracking initialization...
✅ [Module 3] Initialized
   - Confirmation frames: 3
   ...

▶️  System started

📡 Frame 0 | Detections: 50 | Tracks: 0 | Latency: 8.32ms
📡 Frame 30 | Detections: 48 | Tracks: 2 | Latency: 7.98ms
📡 Frame 60 | Detections: 52 | Tracks: 3 | Latency: 8.15ms

============================================================
📊 PERFORMANCE METRICS
============================================================
├─ FPS:                    33.2
├─ Avg Latency:            30.12 ms
├─ Max Latency:            35.45 ms
├─ Active Tracks:          3 (Confirmed: 3, Tentative: 0)
├─ Detections (last):      52
├─ Clusters (last):        8
├─ CPU Usage:              45.3 %
└─ Memory:                 52.1 MB
============================================================

✅ Tracking completed successfully
```

---

## 🎯 다음 단계

1. **실제 센서 연결**
   - Preprocessor::InitializeSensor() 구현
   - UART/USB 드라이버 통합
   - TI 프로토콜 파서 완성

2. **매개변수 최적화**
   - config/default_config.yaml 에서 조정
   - 실제 시나리오에서 테스트
   - FPS 30 이상 달성 확인

3. **배포**
   - Jetson Nano에 포팅
   - 실시간 성능 검증
   - 정규화된 로깅 추가

---

**빌드 명령어 요약:**
```bash
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -j$(nproc)
./radar_tracking_main
```

**기대 성능:**
- FPS: 30+ (★★★)
- 지연시간: <33ms (★★★)
- CPU: <50% (★★★)
- 메모리: ~50MB (★★★)
