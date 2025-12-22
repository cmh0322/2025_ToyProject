#include <ESP32Servo.h>
#include <math.h>
#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

// MCP2515 SPI 핀 (ESP32-C3 기준 예시)
#define PIN_SCK   4
#define PIN_MISO  5
#define PIN_MOSI  6
#define PIN_CS    7

// 서보 핀 (SPI와 절대 겹치면 안 됨!)
const int H_SERVO_PIN = 2;  // 수평 서보
const int V_SERVO_PIN = 3;  // 수직 서보

Servo horizontal_servo;
Servo vertical_servo;

MCP2515 mcp2515(PIN_CS);
struct can_frame rxFrame;
struct can_frame txFrame;

// 기준 각도 (초기값)
float base_hori = 96.0f;
float base_vert = 150.0f;

float hori = 96.0f;
float vert = 150.0f;

int hori_flag = 0; // 스캔 방향: 0 → 증가, 1 → 감소
int vert_flag = 0;

// P 제어 계수 (상황에 맞게 튜닝)
float Kp_x = 1.5f;
float Kp_y = 1.5f;

// 데드존 (정규화 에러 기준)
const float DEADZONE = 0.07f;  // |err| < 0.07이면 0으로 봄

// 한 주기당 서보 최대 이동각
const float MAX_STEP = 1.0f;   // 1도

// 제어 루프 주기 (ms)
const unsigned long CONTROL_PERIOD_MS = 20;

// 최신 에러값
float latest_err_x = 0.0f;
float latest_err_y = 0.0f;
bool  have_error   = false;
bool  detected     = false;    // 목표 인식 여부

int   not_detect_count = 0;    // 연속 not detected 카운트

unsigned long last_control_ms = 0;

// -----------------------------
// 유틸 함수들
// -----------------------------
void reset_txFrame() {
  for (int i = 0; i < txFrame.can_dlc; i++) {
    txFrame.data[i] = 0x00;
  }
}

void send_txFrame() {
  if (mcp2515.sendMessage(&txFrame) != MCP2515::ERROR_OK) {
    Serial.println("CAN send Fail");
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // ============================
  // MCP2515 초기화
  // ============================
  pinMode(PIN_CS, OUTPUT);
  pinMode(H_SERVO_PIN, OUTPUT);
  pinMode(V_SERVO_PIN, OUTPUT);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);  // 모듈 클럭 8MHz 기준
  mcp2515.setNormalMode();                    // 반드시 Normal mode

  // 이 슬레이브가 보내는 프레임: 서보 상태 (ID 0x124)
  txFrame.can_id  = 0x124;   // 11-bit Standard CAN ID
  txFrame.can_dlc = 2;       // [0]: header, [1]: sx

  // ============================
  // 서보 초기화 (ESP32 계열 권장 패턴)
  // ============================
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  horizontal_servo.setPeriodHertz(50);  // 50Hz (20ms)
  vertical_servo.setPeriodHertz(50);

  // 500~2500us 펄스폭 기준 (서보에 맞게 조정 가능)
  horizontal_servo.attach(H_SERVO_PIN, 500, 2500);
  vertical_servo.attach(V_SERVO_PIN, 500, 2500);

  // 초기 위치
  hori = base_hori;
  vert = base_vert;
  horizontal_servo.write((int)hori);
  vertical_servo.write((int)vert);
  delay(1500);

  last_control_ms = millis();

  

  Serial.println("Slave setup done");
}

// bounding box CAN ID : 0x123 (MASTER → SLAVE)
// servo status CAN ID : 0x124 (SLAVE → MASTER)
void loop() {
  // ============================
  // 1) CAN 수신: 0x123 (Bounding Box 에러)
  // ============================
  
  if (mcp2515.readMessage(&rxFrame) == MCP2515::ERROR_OK) {
    if (rxFrame.can_id == 0x123) {//from master
      uint8_t header = rxFrame.data[0];
      int8_t  sx     = (int8_t)rxFrame.data[1];
      int8_t  sy     = (int8_t)rxFrame.data[2];

      Serial.print("RX 0x123 header=");
      Serial.print(header);
      Serial.print(" sx=");
      Serial.print(sx);
      Serial.println();
      //Serial.print(" sy=");
      //Serial.println(sy);

      //객체탐지 되었냐 안 되었냐를 구분
      if (header == 0x00) { // not detected
        not_detect_count++;
        if (not_detect_count >= 5) {
          detected   = false;
          have_error = false;  // 이전 에러값 무효화
        }
      } else { // detected
        not_detect_count = 0;
        detected = true;

        latest_err_x = sx / 100.0f;  // -1.0 ~ 1.0
        latest_err_y = sy / 100.0f;
        have_error   = true;
      }
    }
  }else{
    Serial.println("FAILED TO RX CAN");
  }
  // ============================
  // 2) 제어 루프: 20ms 주기
  // ============================
  unsigned long now = millis();
  if (now - last_control_ms < CONTROL_PERIOD_MS) {
    return;
  }
  last_control_ms = now;

  // ============================
  // 3) 스캔 모드 / P제어 모드 결정
  // ============================
  if (!detected) {
    // ---- 스캔 모드 ----
    float scan_step = 0.5f;  // 좌우 스캔 속도 (deg/step)

    if (hori_flag == 0) { // 증가 방향
      hori += scan_step;
      if (hori >= 180.0f) {
        hori = 180.0f;
        hori_flag = 1;
      }
    } else { // 감소 방향
      hori -= scan_step;
      if (hori <= 0.0f) {
        hori = 0.0f;
        hori_flag = 0;
      }
    }

  } else if (have_error) {
    // ---- P 제어 모드 ----
    float ex = latest_err_x;
    float ey = latest_err_y;

    // 데드존 적용
    if (fabs(ex) < DEADZONE) ex = 0.0f;
    if (fabs(ey) < DEADZONE) ey = 0.0f;

    float dx = Kp_x * ex;
    float dy = Kp_y * ey;

    // 한 스텝당 최대 이동각 제한
    if (dx >  MAX_STEP) dx =  MAX_STEP;
    if (dx < -MAX_STEP) dx = -MAX_STEP;
    if (dy >  MAX_STEP) dy =  MAX_STEP;
    if (dy < -MAX_STEP) dy = -MAX_STEP;

    // 에러 방향에 따라 각도 업데이트
    hori += dx;
    vert += dy;
  }

  // ============================
  // 4) 각도 제한 + 상태 플래그 설정
  // ============================
  //reset_txFrame();

  if (hori < 0.0f) {
    hori = 0.0f;
    //txFrame.data[0] = 0x01; // hori 최소 한계 도달
  }
  if (hori > 180.0f) {
    hori = 180.0f;
    //txFrame.data[1] = 0x01; // hori 최대 한계 도달
  }
  if (vert < 0.0f) {
    vert = 0.0f;
    //txFrame.data[2] = 0x01; // vert 최소 한계 도달
  }
  if (vert > 180.0f) {
    vert = 180.0f;
    //txFrame.data[3] = 0x01; // vert 최대 한계 도달
  }
  //txFrame.data[0] = detected;
  //txFrame.data[1] = (int)((fabs(latest_err_x) < DEADZONE ? 0.0f : latest_err_x) * 100);

  // ============================
  // 5) 상태 CAN 송신 + 서보 반영
  // ============================
  //send_txFrame();
  //Serial.println(latest_err_x);
  //Serial.printf("Sent: header: %d, sx: %d\n", txFrame.data[0], txFrame.data[1]);

  horizontal_servo.write((int)hori);
  vertical_servo.write((int)vert);
}