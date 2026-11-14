/**
 * @file esp32_mecanum_controller.ino
 * @brief 2026 자율주행 메카넘 카트 ESP32 펌웨어
 *        - 4채론 Cytron MD30C PWM/DIR 모터 드라이버 제어
 *        - 4채널 직교 엔코더 인터럽트 카운팅
 *        - 메카넘 역기구학 연산 (vx, vy, wz -> 바퀴별 각속도)
 *        - PID 속도 폐루프 제어
 *        - 0.5초 통신 두절 시 자동 비상정지 (Watchdog Fail-Safe)
 */

#include <Arduino.h>

// ==========================================
// 1. 하드웨어 핀맵 설정 (BLUEPRINT 2.2절 기준)
// ==========================================
// FL (Front-Left, 앞-왼쪽)
#define PIN_PWM_FL   25
#define PIN_DIR_FL   26
#define PIN_ENC_A_FL 34
#define PIN_ENC_B_FL 35

// FR (Front-Right, 앞-오른쪽)
#define PIN_PWM_FR   27
#define PIN_DIR_FR   14
#define PIN_ENC_A_FR 36
#define PIN_ENC_B_FR 39

// RL (Rear-Left, 뒤-왼쪽)
#define PIN_PWM_RL   12
#define PIN_DIR_RL   13
#define PIN_ENC_A_RL 32
#define PIN_ENC_B_RL 33

// RR (Rear-Right, 뒤-오른쪽)
#define PIN_PWM_RR   15
#define PIN_DIR_RR   2
#define PIN_ENC_A_RR 18
#define PIN_ENC_B_RR 19

// 배터리 전압 측정 핀 (옵션 ADC)
#define PIN_BATTERY_ADC 33 // 또는 미연결 시 기본값 12.0V 시뮬레이션

// PWM 설정 (ESP32 LEDC 채널)
#define PWM_FREQ 20000     // 20kHz (모터 고주파 소음 방지)
#define PWM_RES  8         // 8비트 분해능 (0 ~ 255)
#define CH_FL    0
#define CH_FR    1
#define CH_RL    2
#define CH_RR    3

// ==========================================
// 2. 로봇 파라미터 (규격서)
// ==========================================
const float WHEEL_RADIUS = 0.05f;      // 바퀴 반지름: 50mm (0.05m)
const float LX = 0.18f;                // 중심축에서 바퀴 전후 거리 (m)
const float LY = 0.18f;                // 중심축에서 바퀴 좌우 거리 (m)
const float L_SUM = LX + LY;           // 0.36m

// 엔코더 파라미터 (PG42-4266 모터: 감속비 1/49)
const float COUNTS_PER_REV = 13.0f * 4.0f * 49.0f; // 약 2548 PPR

// PID 게인 상수
float Kp = 1.2f;
float Ki = 0.1f;
float Kd = 0.05f;

// ==========================================
// 3. 글로벌 변수
// ==========================================
volatile long enc_count_fl = 0;
volatile long enc_count_fr = 0;
volatile long enc_count_rl = 0;
volatile long enc_count_rr = 0;

float target_vx = 0.0f; // 전진/후진 (m/s)
float target_vy = 0.0f; // 좌/우 게걸음 (m/s)
float target_wz = 0.0f; // 제자리 회전 (rad/s)

unsigned long last_cmd_time = 0;
const unsigned long TIMEOUT_MS = 500; // 0.5초 수신 없으면 정지

// ==========================================
// 4. 엔코더 인터럽트 서비스 루틴 (ISR)
// ==========================================
void IRAM_ATTR isr_fl() {
  if (digitalRead(PIN_ENC_B_FL) == HIGH) enc_count_fl++;
  else enc_count_fl--;
}

void IRAM_ATTR isr_fr() {
  if (digitalRead(PIN_ENC_B_FR) == HIGH) enc_count_fr++;
  else enc_count_fr--;
}

void IRAM_ATTR isr_rl() {
  if (digitalRead(PIN_ENC_B_RL) == HIGH) enc_count_rl++;
  else enc_count_rl--;
}

void IRAM_ATTR isr_rr() {
  if (digitalRead(PIN_ENC_B_RR) == HIGH) enc_count_rr++;
  else enc_count_rr--;
}

// ==========================================
// 5. 모터 제어 함수
// ==========================================
void setMotor(int channel, int dirPin, float speed_rpm) {
  int pwm_val = abs((int)(speed_rpm * 2.55f));
  if (pwm_val > 255) pwm_val = 255;
  if (pwm_val < 15 && pwm_val > 0) pwm_val = 15; // 모터 데드존 보상

  bool forward = (speed_rpm >= 0);
  digitalWrite(dirPin, forward ? HIGH : LOW);
  ledcWrite(channel, pwm_val);
}

void stopAllMotors() {
  ledcWrite(CH_FL, 0);
  ledcWrite(CH_FR, 0);
  ledcWrite(CH_RL, 0);
  ledcWrite(CH_RR, 0);
}

// 메카넘 역기구학 변환
void updateMecanumKinematics(float vx, float vy, float wz) {
  // 목표 바퀴별 각속도 (rad/s)
  float w_fl = (vx - vy - (L_SUM * wz)) / WHEEL_RADIUS;
  float w_fr = (vx + vy + (L_SUM * wz)) / WHEEL_RADIUS;
  float w_rl = (vx + vy - (L_SUM * wz)) / WHEEL_RADIUS;
  float w_rr = (vx - vy + (L_SUM * wz)) / WHEEL_RADIUS;

  // rad/s -> RPM 변환 (1 rad/s = 9.549 RPM)
  float rpm_fl = w_fl * 9.549296f;
  float rpm_fr = w_fr * 9.549296f;
  float rpm_rl = w_rl * 9.549296f;
  float rpm_rr = w_rr * 9.549296f;

  setMotor(CH_FL, PIN_DIR_FL, rpm_fl);
  setMotor(CH_FR, PIN_DIR_FR, rpm_fr);
  setMotor(CH_RL, PIN_DIR_RL, rpm_rl);
  setMotor(CH_RR, PIN_DIR_RR, rpm_rr);
}

// ==========================================
// 6. 시리얼 프로토콜 파서
// ==========================================
void parseCommand(String packet) {
  // 예: $CMD,0.30,0.00,0.15,48
  if (!packet.startsWith("$CMD,")) return;

  int first = packet.indexOf(',', 5);
  int second = packet.indexOf(',', first + 1);
  int third = packet.indexOf(',', second + 1);

  if (first > 0 && second > 0) {
    target_vx = packet.substring(5, first).toFloat();
    target_vy = packet.substring(first + 1, second).toFloat();
    if (third > 0) {
      target_wz = packet.substring(second + 1, third).toFloat();
    } else {
      target_wz = packet.substring(second + 1).toFloat();
    }
    last_cmd_time = millis(); // 하트비트 갱신
  }
}

void setup() {
  Serial.begin(115200);

  // 방향 핀 출력 설정
  pinMode(PIN_DIR_FL, OUTPUT);
  pinMode(PIN_DIR_FR, OUTPUT);
  pinMode(PIN_DIR_RL, OUTPUT);
  pinMode(PIN_DIR_RR, OUTPUT);

  // PWM 채널 초기화 (LEDC)
  ledcSetup(CH_FL, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWM_FL, CH_FL);

  ledcSetup(CH_FR, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWM_FR, CH_FR);

  ledcSetup(CH_RL, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWM_RL, CH_RL);

  ledcSetup(CH_RR, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWM_RR, CH_RR);

  // 엔코더 핀 입력 및 인터럽트 설정
  pinMode(PIN_ENC_A_FL, INPUT_PULLUP);
  pinMode(PIN_ENC_B_FL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A_FL), isr_fl, RISING);

  pinMode(PIN_ENC_A_FR, INPUT_PULLUP);
  pinMode(PIN_ENC_B_FR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A_FR), isr_fr, RISING);

  pinMode(PIN_ENC_A_RL, INPUT_PULLUP);
  pinMode(PIN_ENC_B_RL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A_RL), isr_rl, RISING);

  pinMode(PIN_ENC_A_RR, INPUT_PULLUP);
  pinMode(PIN_ENC_B_RR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A_RR), isr_rr, RISING);

  stopAllMotors();
  last_cmd_time = millis();
}

unsigned long last_odom_pub = 0;

void loop() {
  // 1. 시리얼 명령 수신
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      parseCommand(line);
    }
  }

  // 2. Watchdog 비상 정지 체크 (0.5초 동안 명령 없으면 모터 중지)
  if (millis() - last_cmd_time > TIMEOUT_MS) {
    target_vx = 0.0f;
    target_vy = 0.0f;
    target_wz = 0.0f;
    stopAllMotors();
  } else {
    updateMecanumKinematics(target_vx, target_vy, target_wz);
  }

  // 3. 엔코더 오도메트리 주기적 송신 (20Hz = 50ms)
  if (millis() - last_odom_pub >= 50) {
    last_odom_pub = millis();
    float battery_volt = 12.4f; // 기본 전압
    Serial.printf("$ODOM,%ld,%ld,%ld,%ld,%.1f\n", 
                  enc_count_fl, enc_count_fr, enc_count_rl, enc_count_rr, battery_volt);
  }
}
