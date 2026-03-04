# IR_armybot_project
로봇팔(m0609)와 IR_Camera를 사용해서 야간사격 프로젝트

# 🎯 ArmyBot Project  
> ROS 2 + AI Vision + 음성 제어 기반 **군 사격 훈련 전 과정 자동화 시스템**

---

## 📌 Overview

ArmyBot은  
기존 군 사격 훈련 과정에서 발생하는 **표적 분석의 수작업 의존성**,  
**탄피 및 탄알집 수거 인력 소모**,  
**훈련 데이터의 비정량적 관리 문제**를 해결하기 위해 개발된  
지능형 로봇 자동화 시스템입니다.

본 프로젝트

: contentReference[oaicite:0]{index=0}의 **M0609 협동로봇**을 기반으로,  

- 🎙 음성 인식 제어 시스템  
- 👁 YOLO 기반 객체 탐지  
- 🧠 VLM 기반 표적 분석  
- 🔌 Arduino 엣지 디바이스 연동  

을 통합하여 **사격 준비 → 격발 감지 → 탄피/탄알집 수거 → 표적 분석 → 결과 저장**까지  
전 과정을 자동화했습니다.

---

# 🎯 Motivation

### 기존 사격 훈련의 한계

- 표적지 분석을 인력이 수동 수행
- 탄피 및 탄알집 정리에 시간 소모
- 사격 결과 데이터의 정량적 축적 어려움
- 훈련 자동화 시스템 부재

### 우리가 해결한 문제

ArmyBot은 다음을 목표로 설계되었습니다:

- 🔄 훈련 프로세스 자동화
- 📊 AI 기반 표적 분석 정량화
- 🤖 로봇을 활용한 물리적 작업 자동 수행
- 🗣 음성 기반 직관적 인터페이스 제공

---

# 🏗 System Architecture

<p align="center">
  <img src="./System_Architecture.png" width="700">
</p>

### 핵심 구성 요소

| 구성 요소 | 역할 |
|------------|--------|
| ROS 2 Humble | 전체 노드 통신 및 시스템 제어 |
| Jarvis Voice Node | 음성 인식 및 명령 트리거 |
| YOLO + RealSense | 탄피 / 탄알집 객체 탐지 |
| Gemini VLM | 표적지 AI 분석 |
| Arduino | 격발 신호 감지 |
| M0609 Robot Arm | 물리적 수거 작업 수행 |
| Flask | User Interface 구현 |

---

# 🔄 System Flow
<img width="979" height="555" alt="image" src="https://github.com/user-attachments/assets/64007740-2875-4ead-b298-214536292968" />


### 동작 순서

1. "자비스, 준비" 음성 명령
2. 로봇 사격 준비 자세 이동
3. Arduino 격발 감지
4. YOLO 객체 탐지
5. Depth → Robot 좌표 변환
6. 로봇 탄피/탄알집 수거
7. 표적지 촬영
8. Gemini VLM 분석
9. 결과 저장

---

# 🛠 Tech Stack

## 🖥 Environment
- Ubuntu 22.04 LTS
- ROS 2 Humble
- Python 3.10
- JavaScript
- HTML

## 🤖 Robotics
- contentReference[oaicite:1]{index=1} M0609
- DSR_ROBOT2 Python API
- Onrobot RG2 Gripper

## 👁 Vision & AI
- `ultralytics` (YOLOv8)
- :contentReference[oaicite:2]{index=2} RealSense D435i
- Gemini API (VLM 분석)
- OpenCV

## 🔌 Embedded
- Arduino (격발 감지)
- PySerial

## 🎙 Voice System
- SpeechRecognition
- gTTS

---

# 👨‍💻 My Contribution

- ROS 2 기반 로봇 제어 노드 설계 및 구현
- RealSense Depth → Robot 좌표계 변환 알고리즘 구현
- YOLO 기반 객체 탐지 파이프라인 구축
- Arduino-ROS2 시리얼 통신 프로토콜 설계
- 음성 명령 기반 자동 실행 로직 설계
- 전체 시스템 통합 및 디버깅

---

# 📊 Key Achievements

- ✅ 사격 후 정리 자동화 구현
- ✅ 객체 탐지 및 로봇 픽업 파이프라인 완성
- ✅ 음성 명령 기반 무인 자동화 시스템 구축
- ✅ Vision-AI-Robot-Embedded 통합 아키텍처 설계 경험 확보

---

# 📂 Project Structure


armybot/

├── robot_control.py

├── yolo_node.py

├── ai_count.py

└── onrobot.py

arduino_bridge/

└── switch_edge_pub.py

jarvis_project/

└── jarvis.py

resource/

├── brass_magazine.pt

├── calibration_matrix.yaml

└── result/

od_msg/srv/

└── SrvDepthPosition.srv

armbot_web/

├── commander.py

├── shooter.py

├── templates/

├───── commander.html

└───── shooter.html

---

# ▶️ How to Run

## Arduino Bridge

cd ir_gunshot_staff

colcon build --packages-select armybot arduino_bridge

source install/setup.bash

ros2 run arduino_bridge switch_edge_pub

## Voice Control System

cd ir_gunshot_staff/src/jarvis_project

python3 jarvis.py

## Armybot
─── Terminal 1 ───

ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.1.100 port:=12345 model:=m0609

─── Terminal 2 ───

ros2 launch realsense2_camera rs_align_depth_launch.py depth_module.depth_profile:=848x480x30 rgb_camera.color_profile:=1280x720x30 initial_reset:=true align_depth.enable:=true enable_rgbd:=true enable_infra:=true enable_infra1:=true enable_infra2:=true depth_module.emitter_enabled:=1 pointcloud.enable:=true

─── Terminal 3 ───

realsense-viewer

└──realsenseIRconfig.json

─── Terminal 4 ───

cd ir_gunshot_staff

colcon build --packages-select armybot

source install/setup.bash

ros2 launch armybot armybot.launch.py

## UI
─── Terminal 5 ───

cd ir_gunshot_staff/src/armbot_web

python3 commander.py

─── Terminal 6 ───

cd ir_gunshot_staff/src/armbot_web

python3 shooter.py

# 💡 What I Learned

ROS 2 기반 분산 노드 아키텍처 설계 경험

Vision-Depth 좌표 변환 실전 적용

로봇 제어에서의 정밀도와 안정성 문제 해결

임베디드-로봇 간 실시간 통신 설계

AI 모델을 실제 물리 시스템에 통합하는 방법

# 🚀 Future Improvements

멀티 타겟 동시 분석 기능

사격 점수 자동 정량화 알고리즘 개선

UI 기반 실시간 모니터링 시스템 구축

실시간 대시보드 시각화

# 👥 Team

ROKEY D-3조 | 삼인용
이강인 · 주진 · 최순일 · 최재형
