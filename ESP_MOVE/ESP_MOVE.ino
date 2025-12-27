//To. Mr. Siyoung
//
//형님 프로젝트에 도움 주셔서 감사합니다!
//일단 라베파에서  0 ~ 5단계로 나누어 봤고요?
//0(제일 멀음) ~ 5(제일 가까움) 으로 했습니다.
//나중에 필요하시면 단계 조절도 가능할 것 같아요!
//
//그리고 모터 제어 함수에 인자를 넘길 때 가중치 곱하는 부분도 업데이트 했습니다
//out_L, out_R이 sx의 값에 따라 바뀌지 않습니까?
//보통 sx값이 실질적으로 -30 ~ 30 사이에 머무릅니다.
//이때 Out값의 공식이 아래와 같더군요..
// OUT = |100 - 0.75sx| +1.5sx
//즉, 모터에 전달되는 Out값은 100 ~ 122.5 사이에요. 참 시원찮죠?
//그래서 공식을 새로 만들어 봤습니다~
// NEWOUT = (OUT - 100) * 4 + 150           (OUT는 세 줄 위에 저 출력입니다)
//이렇게 하면 OUT값이 100 ~ 122.5일 때, NEWOUT값은 150 ~ 250이 됩니다
//함수에 입력하기 전에 -255 ~ 255 클램핑 할거니까 걱정 말으셔요
//
//Line 191에 가중치 추가 코드 있습니다
//Line 234에 바운딩박스 레벨 반영 감속 로직 있습니다
//바운딩박스 레벨 * 20만큼 감속해서 제어됩니다.
//그러면 범위가 최대 150 ~ 255 -> 70 ~ 175가 되겠네요

//라베파에서 패킷 전송할 때, data[2]에 바운딩박스 레벨 info만 담아주세요~!

#include <SPI.h>
#include <mcp2515.h>
#include <Arduino.h>

#if CONFIG_FREERTOS_UNICORE
    static const BaseType_t app_cpu = 0;
#else
    static const BaseType_t app_cpu = 1;
#endif

// MCP2515 SPI Pins
#define PIN_SCK   4
#define PIN_MISO  5
#define PIN_MOSI  6
#define PIN_CS    7
#define PIN_INT   2

//DC Motor Pins
const int LEFT_EN = GPIO_NUM_20;
const int LEFT_PINS[2] = {
    GPIO_NUM_0,
    GPIO_NUM_1
};
const int RIGHT_EN = GPIO_NUM_21;
const int RIGHT_PINS[2] = {
    GPIO_NUM_2,
    GPIO_NUM_3
};

MCP2515 mcp2515(PIN_CS);

void controlMotor(int16_t t_L, int16_t t_R){
    if(t_L < 0){
      analogWrite(LEFT_EN, (uint8_t)(-t_L));  // 명확한 부호 반전
      digitalWrite(LEFT_PINS[0], LOW);
      digitalWrite(LEFT_PINS[1], HIGH);
    }else{
      analogWrite(LEFT_EN, (uint8_t)t_L);
      digitalWrite(LEFT_PINS[0], HIGH);
      digitalWrite(LEFT_PINS[1], LOW);
    }

    if(t_R < 0){
      analogWrite(RIGHT_EN, (uint8_t)(-t_R));  // 명확한 부호 반전
      digitalWrite(RIGHT_PINS[0], LOW);
      digitalWrite(RIGHT_PINS[1], HIGH);
    }else{
      analogWrite(RIGHT_EN, (uint8_t)t_R);
      digitalWrite(RIGHT_PINS[0], HIGH);
      digitalWrite(RIGHT_PINS[1], LOW);
    }
}

//globals
const uint16_t DIST_DANGER = 40;    //장애물이 40cm 이내면 회피

//GAINS - 실험적 조정 필요함
const float K_TARGET    = 1.5f;     //타겟 추적 시 게인값
const float K_AVOID     = 4.0f;     //장애물 피할 때는 더 급하게 꺾어
const float BASE_SPEED  = 100.0f;

//(Task_Can_Rx -> Task_Navigation)
struct SensorData {
    //초음파센서 MCU (ID: 0x125)
    uint16_t dist_left;             //2 ~ 400
    uint16_t dist_right;            //2 ~ 400
    uint16_t dist_back;             //2 ~ 400

    //라베파 카메라(ID: 0x124)
    int8_t vision_error_sx;         //-70 ~ 70
    bool detected;                  //0 ~ 1

    //라베파 바운딩박스 크기 단계
    uint8_t bbox_size_level;        //0 ~ 5
};

//(Task_Navigation -> Task_Motor_Drive)
struct MotorCommand {
    int16_t out_L;
    int16_t out_R;

    //바운딩박스 크기
    uint8_t bbox_size_level;        //0 ~ 5
};

// Queue 핸들
QueueHandle_t xQueueSensorData;
QueueHandle_t xQueueMotorCommand;

//Tasks
void Task_Can_Rx(void* params){
    struct can_frame rxFrame;
    SensorData localData = {400, 400, 400, 0, false, 0};
    
    Serial.println("can task");
    while(1){
        if(mcp2515.readMessage(&rxFrame) == MCP2515::ERROR_OK){
            
            if(rxFrame.can_id == 0x123){ // 카메라 데이터
                
                Serial.printf("[@MASTER]ID: %x, Header: %d, sx: %d, bboxLevel: %d\n", rxFrame.can_id, (int8_t)rxFrame.data[0], (int8_t)rxFrame.data[1], (uint8+_t)rxFrame.data[2]);

                bool is_detected = (rxFrame.data[0] == 1);
                localData.detected = is_detected;

                if(is_detected){
                    localData.vision_error_sx = (int8_t)rxFrame.data[1];
                    localData.bbox_size_level = (uint8_t)rxFrame.data[2];
                }
                else{
                    localData.vision_error_sx = 0;
                    localData.bbox_size_level = 0;
                }
                
                xQueueSend(xQueueSensorData, &localData, 0);
            }
            else if(rxFrame.can_id == 0x125){   //초음파센서 데이터
                localData.dist_back  = (rxFrame.data[2] << 8) | rxFrame.data[3];
                localData.dist_left = (rxFrame.data[4] << 8) | rxFrame.data[5];
                localData.dist_right  = (rxFrame.data[6] << 8) | rxFrame.data[7];

                Serial.printf("[@DETECT]ID: %x, dist_left: %3d, dist_right: %3d, dist_back: %3d\n", rxFrame.can_id, localData.dist_left, localData.dist_right, localData.dist_back);
                
                xQueueSend(xQueueSensorData, &localData, 0);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

//주행 알고리즘
void Task_Navigation(void* params){
    SensorData receivedData = {400, 400, 400, 0, false, 0};
    MotorCommand motorCmd = {0, 0};
    
    while(1){
        // Queue에서 센서 데이터 수신
        if(xQueueReceive(xQueueSensorData, &receivedData, portMAX_DELAY) == pdTRUE){
            Serial.printf("sx: %d, dl: %d, dr: %d, det: %d\n", 
            receivedData.vision_error_sx, receivedData.dist_left, receivedData.dist_right, receivedData.detected);
            int8_t sx = receivedData.vision_error_sx;
            uint16_t d_left = receivedData.dist_left;      // 타입 일관성
            uint16_t d_right = receivedData.dist_right;
            bool is_detected = receivedData.detected;
            uint8_t bbox_level = receivedData.bbox_size_level;

            float out_L = 0.0f;
            float out_R = 0.0f;

            // 알고리즘 적용
            if(is_detected){
                /* === [인력] 객체 추적 (Target Force) === */
                float force_target = (float)sx; 

                /* === [척력] 장애물 회피 (Avoidance Force) === */
                float force_avoid = 0.0f;
                if (d_left < DIST_DANGER)    force_avoid += (DIST_DANGER - d_left) * K_AVOID;
                if (d_right < DIST_DANGER)   force_avoid -= (DIST_DANGER - d_right) * K_AVOID;

                float steering = (force_target * K_TARGET) + force_avoid;
                float current_speed = BASE_SPEED;
                
                current_speed -= fabs(steering) * 0.5f; 
                
                if(current_speed < 0.0f) current_speed = 0.0f; 

                if(d_left < 10 || d_right < 10) current_speed = 0.0f;

                float mix_L = current_speed + steering;
                float mix_R = current_speed - steering;

                //가중치 조정 - 새로 추가됨!
                mix_L = (mix_L - 100) * 4.0f + 150.0f; 
                mix_R = (mix_R - 100) * 4.0f + 150.0f; 

                // 클램핑
                if (mix_L > 255.0f) mix_L = 255.0f;
                if (mix_L < -255.0f) mix_L = -255.0f;

                if (mix_R > 255.0f) mix_R = 255.0f;
                if (mix_R < -255.0f) mix_R = -255.0f;

                out_L = mix_L;
                out_R = mix_R;

            }
            else{// 객체 미감지 시 정지
                out_L = 0.0f;
                out_R = 0.0f;
            }

            // 모터 명령 구조체 생성 및 Queue로 전송
            // float → int16_t 변환 (이미 ±255 범위로 제한되어 안전)
            motorCmd.out_L = (int16_t)out_L;
            motorCmd.out_R = (int16_t)out_R;
            Serial.printf("outL: %d, outR: %d\n", 
            motorCmd.out_L, motorCmd.out_R);
            xQueueSend(xQueueMotorCommand, &motorCmd, 0);
        }
    }
}

//모터 구동
void Task_Motor_Drive(void* params){
    MotorCommand motorCmd = {0, 0, 0};  // 초기값 명시
    
    while(1){
      // Serial.println("Motor");
        // Queue에서 모터 명령 수신
        if(xQueueReceive(xQueueMotorCommand, &motorCmd, portMAX_DELAY) == pdTRUE){
            // 모터 제어
            //바운딩박스 레벨 제어

            motorCmd.out_L -= (int16_t)(motorCmd.bbox_size_level * 20);
            motorCmd.out_R -= (int16_t)(motorCmd.bbox_size_level * 20);

            Serial.printf("outL: %d, outR: %d\n", motorCmd.out_L, motorCmd.out_R);

            controlMotor(motorCmd.out_L, motorCmd.out_R);
        }
    }
}

void setup(){
    Serial.begin(115200);
    vTaskDelay(100 /portTICK_PERIOD_MS);

    //can초기화
    SPI.begin();
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();

    //pin 초기화
    pinMode(LEFT_EN,  OUTPUT);
    pinMode(RIGHT_EN, OUTPUT);

    for (int i = 0; i < 2; i++) {
        pinMode(LEFT_PINS[i], OUTPUT);
        pinMode(RIGHT_PINS[i], OUTPUT);
    }

    // Queue 생성
    // SensorData: 크기 10 - CAN 버스트 수신 대비 (0x124, 0x125가 연속으로 올 수 있음)
    // MotorCommand: 크기 3 - Navigation이 Motor보다 빠를 때 대비
    xQueueSensorData = xQueueCreate(10, sizeof(SensorData));
    xQueueMotorCommand = xQueueCreate(3, sizeof(MotorCommand));

    //디버깅용
    xTaskCreatePinnedToCore( Task_Can_Rx, "Can_Rx", 4096, NULL, 3, NULL, app_cpu);
    xTaskCreatePinnedToCore( Task_Navigation, "Navigation", 4096, NULL, 2, NULL, app_cpu);
    xTaskCreatePinnedToCore( Task_Motor_Drive, "Motor_Drive", 2048, NULL, 1, NULL, app_cpu);

    Serial.println("System Started With Queue-Based Architecture");

    vTaskDelete(NULL);
}

void loop(){

}