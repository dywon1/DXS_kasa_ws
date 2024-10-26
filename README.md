![대회장항공뷰](https://github.com/user-attachments/assets/d32a9c5b-853f-400f-be54-fc8f909b2e9a)

# 한국교통안전공단 주관 대학생 창작 자율주행 자동차대회
## KASA 프로젝트 포트폴리오

본 저장소는 자율주행 차량 개발을 위한 세 가지 주요 패키지를 포함하고 있습니다:

1. **lidar**: LiDAR 센서 데이터 처리 및 장애물 감지
2. **path_planning**: 경로 계획 및 경로 생성 알고리즘
3. **control**: 차량의 종방향 및 횡방향 제어 알고리즘

각 패키지는 자율주행 차량의 특정 기능을 담당하며, 전체적으로 통합되어 자율주행 시스템을 구성합니다.

---

## 목차

- [프로젝트 소개](#프로젝트-소개)
- [패키지 구성](#패키지-구성)
  - [1. lidar](#1-lidar)
  - [2. path_planning](#2-path_planning)
  - [3. control](#3-control)
- [시스템 요구 사항](#시스템-요구-사항)
- [사용 방법](#사용-방법)
- [데모](#데모)
- [프로젝트 구조](#프로젝트-구조)
- [문의](#문의)

---

## 프로젝트 소개

이 프로젝트는 자율주행 차량 개발을 위한 핵심적인 기능들을 구현한 ROS2 HUMBLE 기반의 패키지 모음입니다. LiDAR 센서로부터 차량 전방의 꼬깔콘 데이터를 수집하고, 수집된 데이터를 기반으로 꼬깔콘 사이의 경로를 계획하며, 차량을 제어하여 목표 지점까지 안전하게 주행할 수 있도록 합니다.

---

## 패키지 구성

### 1. **lidar**

LiDAR 센서로부터 수집된 포인트 클라우드 데이터를 처리하여 차량 전방의 꼬깔콘을 감지하고, RANSAC 및 필터링을 통해 노이즈를 제거하여 최적화합니다.

**주요 기능:**

- sampling.cpp: voxel_grid downsample을 통해 라이다 데이터 양을 줄이고 노이즈를 제거하여 최적화한다.
- roi.cpp: 차량 전방의 원하는 부분만 잘라서 보게 한다.
- ransac.cpp: 아웃라이어 제거 & 지면 제거를 통해 꼬깔콘만 남게 한다.
- parameters.yaml: 위 3 코드의 파라미터를 조정한다.

### 2. **path_planning**

차량 전방의 꼬깔콘 PCD를 받고 클러스터링 및 CCW 알고리즘을 통해 꼬깔콘의 사이의 중점 경로를 생성합니다.

**주요 기능:**

- /ransac_points를 받고 꼬깔콘들을 clustering 알고리즘과 Eigen 을 통해 꼬깔콘을 한 점으로 만든다.
- 인지할 콘 크기 규격을 제한해 사람 또는 잔디 등 꼬깔콘 외의 장애물을 인지하지 않도록 한다.
- CCW 알고리즘을 통해 차량에서 멀리 떨어져 있는 경로까지 계산한다.
- marker_array와 float32_multi_array를 통해 시각화와 경로 데이터를 전달한다.

### 3. **control**

계획된 경로를 따라 차량을 제어하는 알고리즘을 제공합니다. 종방향 제어(속도 제어)와 횡방향 제어(조향각 제어)를 포함합니다.

**주요 기능:**

- PID 제어기를 이용한 속도 제어
- Pure Pursuit 및 Stanley 제어기를 이용한 조향각 제어
- 곡률에 따른 동적 속도 조절
- 센서 데이터 피드백을 통한 제어기 튜닝

---

## 시스템 요구 사항

- **ubuntu 22.04**
- **ROS2 Humble Hawksbill**
- **C++14** 이상
- **Python 3.10** 이상
- Ouster OS1-32 LiDAR 센서 드라이버 (humble 버전)

---

## 사용 방법

### 1. LiDAR 데이터 수집 및 처리

LiDAR 센서 드라이버를 실행하고, `lidar`를 실행하여 라이다 데이터를 최적화합니다.

```bash
# lidar 패키지 실행
ros2 launch lidar lidar.launch.py
```

설정된 Rviz2가 켜지면서 시각화할 수 있습니다.

### 2. 경로 계획 실행

환경 데이터가 수집되면, `path_planning`를 실행하여 경로를 계획합니다.

```bash
ros2 run path_planning path_planning
```

### 3. 차량 제어 실행

계획된 경로를 기반으로 차량을 제어합니다.

```bash
ros2 run control_package control_node
```


---

## 데모

![데모 영상 썸네일](link_to_demo_video_thumbnail)

[데모 영상 보기](link_to_demo_video)

---

## 프로젝트 구조

```
ros2_ws/
├── src/
│   ├── lidar/
│   ├── path_planninge/
│   └── control/
├── install/
├── build/
└── log/
```

- `lidar_package`: LiDAR 데이터 처리 패키지
- `path_planning_package`: 경로 계획 패키지
- `control_package`: 차량 제어 패키지


---

## 문의

프로젝트에 대한 문의나 제안 사항이 있으시면 아래로 연락주시기 바랍니다.

- **Email**: wdy9876@koreatech.ac.kr

---

**감사합니다!**

