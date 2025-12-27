#include <math.h>
#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

#define PIN_SCK   14
#define PIN_MISO  12
#define PIN_MOSI  13
#define PIN_CS    15
#define PIN_INT   4   // MCP2515 INT 핀

MCP2515 mcp2515(PIN_CS);
struct can_frame rxFrame;
struct can_frame txFrame;

// MCP2515 INT용 플래그
volatile bool canIntFlag = false;

// ISR (ESP8266에서는 ICACHE_RAM_ATTR)
ICACHE_RAM_ATTR void onCanInt() {
    // 여기선 절대 SPI 통신하지 말고, 플래그만!
    canIntFlag = true;
}

void setup() {
    Serial.begin(115200);      // Python과 연결된 UART
    delay(1000);
    Serial.println("MASTER: UART <-> CAN, INT RX for 0x123");

    pinMode(PIN_CS, OUTPUT);
    SPI.begin();               // D5, D6, D7 사용

    // MCP2515 초기화
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);  // SLAVE랑 동일하게
    mcp2515.setNormalMode();

    // MASTER → SLAVE 명령 프레임 기본값 (0x124)
    txFrame.can_id  = 0x123;
    txFrame.can_dlc = 3;   // header, sx, sy 3바이트

    // INT 핀 인터럽트 설정
    pinMode(PIN_INT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_INT), onCanInt, FALLING);

    Serial.println("MASTER init done");
}

void loop() {
    /* ===================== 1) Python → CAN(0x124) 송신 ===================== */

    // Python에서 3바이트씩: [header, sx, sy]
    while (Serial.available() >= 3) {
        uint8_t header = Serial.read();
        int8_t  sx     = (int8_t)Serial.read();
        int8_t  sy     = (int8_t)Serial.read();

        txFrame.data[0] = header;
        txFrame.data[1] = sx;
        txFrame.data[2] = sy;

        MCP2515::ERROR txErr = mcp2515.sendMessage(&txFrame);
        if (txErr == MCP2515::ERROR_OK) {
            Serial.print("[MASTER] TX CMD 0x123: header=");
            Serial.print((int8_t)txFrame.data[0]);
            Serial.print(", sx=");
            Serial.print((int8_t)txFrame.data[1]);
            Serial.print(", sy=");
            Serial.println(txFrame.data[2]);
        } else {
            Serial.print("[MASTER] CAN send ERROR: ");
            Serial.println(txErr);
        }
    }

    /* ===================== 2) SLAVE → 0x123 응답 INT 기반 수신 ===================== */

    if (canIntFlag) {
        canIntFlag = false;  // 플래그 클리어

        // MCP2515는 INT가 low로 유지될 수 있으므로,
        // 버퍼에 있는 메시지를 다 읽을 때까지 반복 처리.
        while (true) {
            MCP2515::ERROR rxErr = mcp2515.readMessage(&rxFrame);
            if (rxErr != MCP2515::ERROR_OK) {
                // 더 이상 읽을 메시지가 없으면 탈출
                break;
            }

            // SLAVE → MASTER 응답 ID: 0x123
            if (rxFrame.can_id == 0x124) {
                // SLAVE 쪽에서 어떤 포맷으로 넣었는지에 맞춰 파싱
                // 예시: resulst_x, result_y 2바이트씩 (int16_t)
                int16_t result_x = (int16_t)(rxFrame.data[0] | (rxFrame.data[1] << 8));
                int16_t result_y = (int16_t)(rxFrame.data[2] | (rxFrame.data[3] << 8));

                Serial.print("[MASTER] RX RESP 0x124: result_x=");
                Serial.print(result_x);
                Serial.print(", result_y=");
                Serial.println(result_y);

                // Python으로 다시 넘기는 프로토콜 (예시)
                // [0xBB, result_x L, result_x H, result_y L, result_y H]
                uint8_t header_resp = 0xBB;
                Serial.write(header_resp);
                Serial.write((uint8_t)(result_x & 0xFF));
                Serial.write((uint8_t)((result_x >> 8) & 0xFF));
                Serial.write((uint8_t)(result_y & 0xFF));
                Serial.write((uint8_t)((result_y >> 8) & 0xFF));
            } else {
                // 다른 ID가 필요하면 여기서도 처리 가능
                /*
                Serial.print("[MASTER] RX other ID: 0x");
                Serial.println(rxFrame.can_id, HEX);
                */
            }
        }
    }

    // 필요하다면 약간 쉼
    delayMicroseconds(10);
}