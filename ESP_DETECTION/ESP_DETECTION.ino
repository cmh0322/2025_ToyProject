#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

#define NUM_ULTRASONICS 3

// ======== PIN 설정 (ESP32-C3) ========
#define PIN_SCK   4
#define PIN_MISO  5
#define PIN_MOSI  6
#define PIN_CS    7

// 초음파 핀 설정
const int TRIG_PIN = 3; 
const int ECHO_PINS[3] = {0, 1, 2}; 

// ======== 센서 방향 정의 (사용자 환경에 맞게 인덱스 수정 필요) ========
#define IDX_BACK  0  // distances[0]이 뒤쪽
#define IDX_LEFT  1  // distances[1]이 왼쪽
#define IDX_RIGHT 2  // distances[2]이 오른쪽

// 장애물 감지 기준 거리 (cm) - 이 거리보다 가까우면 장애물로 판단
#define OBSTACLE_THRESHOLD 50 

MCP2515 mcp2515(PIN_CS);
struct can_frame txFrame;

// 인터럽트 동기화를 위한 뮤텍스 (원본 코드에 정의가 빠져있어서 추가함)
portMUX_TYPE echoMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t echo_start[NUM_ULTRASONICS] = {0};
volatile uint32_t echo_end[NUM_ULTRASONICS]   = {0};
volatile bool     echo_done[NUM_ULTRASONICS]  = {false};
volatile uint32_t distances[NUM_ULTRASONICS]  = {0}; // 계산된 거리 저장

// 인터럽트 서비스 루틴
inline void IRAM_ATTR echo_isr_common(int idx){
  uint32_t now = micros();
  int level = digitalRead(ECHO_PINS[idx]);

  portENTER_CRITICAL_ISR(&echoMux);
  if(level == HIGH){
    echo_start[idx] = now;
    echo_done[idx] = false;
  }else{
    echo_end[idx] = now;
    echo_done[idx] = true;
  }
  portEXIT_CRITICAL_ISR(&echoMux);
}

void IRAM_ATTR echo_isr_1() { echo_isr_common(1); }
void IRAM_ATTR echo_isr_0() { echo_isr_common(0); }
void IRAM_ATTR echo_isr_2() { echo_isr_common(2); }

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("\n=== CAN Transmitter : Protocol Logic Applied ===");

    // SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);/
    SPI.begin();

    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();

    txFrame.can_id = 0x125;
    txFrame.can_dlc = 8; // 프로토콜에 따라 8바이트 사용

    pinMode(TRIG_PIN, OUTPUT);
    digitalWrite(TRIG_PIN, LOW);

    for (int i = 0; i < NUM_ULTRASONICS; i++) {
        pinMode(ECHO_PINS[i], INPUT);
    }

    attachInterrupt(digitalPinToInterrupt(ECHO_PINS[0]), echo_isr_0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ECHO_PINS[1]), echo_isr_1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ECHO_PINS[2]), echo_isr_2, CHANGE);
}

void loop(){
    // 1. 초음파 측정 시작 (Trigger)
    for(int i = 0; i < NUM_ULTRASONICS; i++){
        echo_done[i] = false;
    }

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    delay(30); // 측정 대기 시간 (충분히 확보)

    // 2. 거리 계산
    uint16_t dist_vals[NUM_ULTRASONICS] = {0, 0, 0}; // 임시 저장용

    for(int i = 0; i < NUM_ULTRASONICS; i++){
      bool done = false;
      uint32_t start_us = 0, end_us = 0;

        portENTER_CRITICAL(&echoMux);
        if(echo_done[i]){
          done = true;
          start_us = echo_start[i];
          end_us = echo_end[i];
          echo_done[i] = false; // flag clear
        }
        portEXIT_CRITICAL(&echoMux);    
        
        if(done && end_us > start_us){
          uint32_t duration = end_us - start_us;
          // 거리가 너무 멀거나 튀는 값은 0 처리 혹은 최대값 제한 필요
          uint32_t cm = duration / 58;
          if(cm > 600) cm = 600; // 예외처리
          distances[i] = cm;
        }
        
        dist_vals[i] = (uint16_t)distances[i];
    }

    // ==============================================
    // 3. 프로토콜 패킹 (Layout 적용)
    // ==============================================
    
    // 센서별 거리 할당
    uint16_t d_back  = dist_vals[IDX_BACK];
    uint16_t d_left  = dist_vals[IDX_LEFT];
    uint16_t d_right = dist_vals[IDX_RIGHT];

    // data[0]: 장애물 감지 여부 (0 or 1)
    // data[1]: 어느 센서? (Bitmask: 001=Back, 010=Left, 100=Right -> 0~7)
    uint8_t detected = 0;
    uint8_t sensors_flag = 0;

    // 감지 로직 (설정된 THRESHOLD보다 가까우면 감지로 판단)
    if(d_back > 0 && d_back < OBSTACLE_THRESHOLD) { 
        detected = 1; 
        sensors_flag |= (1 << 0); // 1 (binary 001)
    }
    if(d_left > 0 && d_left < OBSTACLE_THRESHOLD) { 
        detected = 1; 
        sensors_flag |= (1 << 1); // 2 (binary 010)
    }
    if(d_right > 0 && d_right < OBSTACLE_THRESHOLD) { 
        detected = 1; 
        sensors_flag |= (1 << 2); // 4 (binary 100)
    }

    txFrame.data[0] = detected;
    txFrame.data[1] = sensors_flag;

    // data[2]~[7]: 거리값 (uint16_t를 2바이트로 분할 - Big Endian 방식)
    // 뒤쪽
    txFrame.data[2] = (uint8_t)(d_back >> 8);   // 상위 바이트
    txFrame.data[3] = (uint8_t)(d_back & 0xFF); // 하위 바이트
    // 왼쪽
    txFrame.data[4] = (uint8_t)(d_left >> 8);
    txFrame.data[5] = (uint8_t)(d_left & 0xFF);
    // 오른쪽
    txFrame.data[6] = (uint8_t)(d_right >> 8);
    txFrame.data[7] = (uint8_t)(d_right & 0xFF);


    // 4. CAN 전송
    if(mcp2515.sendMessage(&txFrame) == MCP2515::ERROR_OK){
        Serial.printf("Sent: Det=%d | SensorFlag=%d | Back=%d | Left=%d | Right=%d\n", 
                      detected, sensors_flag, d_back, d_left, d_right);
    }else{
        Serial.println("Failed to send");
    }
}