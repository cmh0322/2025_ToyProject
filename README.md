<div align="center">

# 🏎️ Vision-Guided Autonomous RC System
### AI-Powered Target Tracking & Obstacle Avoidance via CAN Bus Network

<p>
  <img src="https://img.shields.io/badge/Raspberry%20Pi%204-A22846?style=for-the-badge&logo=raspberrypi&logoColor=white"/>
  <img src="https://img.shields.io/badge/ESP32--C3-E74C3C?style=for-the-badge&logo=espressif&logoColor=white"/>
  <img src="https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white"/>
  <img src="https://img.shields.io/badge/TensorFlow%20Lite-FF6F00?style=for-the-badge&logo=tensorflow&logoColor=white"/>
  <img src="https://img.shields.io/badge/CAN%20Bus-00529B?style=for-the-badge&logo=connectivity&logoColor=white"/>
  <img src="https://img.shields.io/badge/FreeRTOS-00A4EF?style=for-the-badge&logo=compuware&logoColor=white"/>
</p>

<br/>

> **"지능형 시각 분석과 분산 제어의 결합"** <br/> 라즈베리파이의 AI 객체 탐지와 CAN 통신 기반 분산 처리를 활용한 자율 주행 RC 플랫폼

[데모 영상 보기(링크)] | [회로도 및 설계서(링크)]

</div>

---

## 🏗️ System Architecture



본 프로젝트는 연산과 제어의 역할을 명확히 분리하여 실시간성을 극대화했습니다.

1. **Vision AP (Raspberry Pi)**: 
    * MobileNet V1 TFLite 모델을 활용한 실시간 객체 탐지
    * 화면 중앙과의 오차값($sx, sy$) 산출 및 UART 전송
2. **Master Node (ESP8266)**: 
    * UART(AP) ↔ CAN(Slaves) 데이터 게이트웨이
    * 수신된 오차 데이터를 CAN ID `0x123` 프레임으로 변환 및 배분
3. **Slave Nodes (ESP32/C3)**: 
    * **Track**: 타겟 추적용 2축 Pan/Tilt 서보 모터 제어 (P-제어)
    * **Detection**: 3방향 초음파 센서 실시간 장애물 모니터링
    * **Move**: 인력-척력 알고리즘 기반 DC 모터 주행 (FreeRTOS 적용)

---

## 🧠 Core Algorithm: Potential Field

타겟으로 향하는 **인력(Attractive)**과 장애물을 피하는 **척력(Repulsive)**을 벡터적으로 합산하여 최적의 조향각을 결정합니다.

$$Steering = (Force_{target} \times K_{target}) + Force_{avoid}$$

* **Target Tracking**: 오차값 $sx$가 커질수록 해당 방향으로 더 강한 인력 발생
* **Obstacle Avoidance**: 초음파 센서 거리 50cm 이내 진입 시 장애물 반대 방향으로 급격한 척력 발생

---

## 🛠️ Tech Stack & Tools

| Category | Technology Stack |
| :--- | :--- |
| **Vision (AP)** | `Python`, `TFLite`, `OpenCV`, `Picamera2` |
| **Distributed** | `CAN Bus (MCP2515)`, `UART`, `SPI` |
| **Embedded** | `ESP32-S3/C3`, `FreeRTOS`, `Arduino Core` |
| **Actuators** | `DC Motors (L298N)`, `SG90 Servos` |

---

## 🚀 Getting Started

1. **Clone the repository**
    ```bash
    git clone [https://github.com/username/vision-rc-system.git](https://github.com/username/vision-rc-system.git)
    ```

2. **Raspberry Pi Environment Setup**
    ```bash
    pip install tflite-runtime opencv-python pyserial
    ```

3. **Firmware Upload**
    각 노드별 MCU에 맞는 `.ino` 파일을 빌드하여 업로드합니다.
    * **Master**: `master.ino` (ESP8266)
    * **Move**: `move_freertos.ino` (ESP32)
    * **Track/Detect**: `slave_control.ino` (ESP32-C3)

---

<div align="center">
  <p>&copy; 2025 Vision RC Project. 건국대학교 전기전자공학부 학사 프로젝트.</p>
</div>
