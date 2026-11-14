/**
 * @file esp32_mecanum_controller.ino
 * @brief 2026 자율주행 메카넘 카트 ESP32 펌웨어 (micro-ROS over USB Serial)
 *
 * - 기준 문서: datasheets/esp32_pinmap_and_hall_sensor_check.md
 * - 빌드 환경: Arduino-ESP32 v2.0.x + micro_ros_arduino
 * - 주요 기능:
 *   1. micro-ROS (USB Serial, UART0):
 *      - Subscribe: /cmd_vel (geometry_msgs/msg/Twist) -> 키보드(wsadqe) 및 Nav2 통합 명령
 *      - Publish:   /wheel/odom (nav_msgs/msg/Odometry) -> EKF(/odometry/filtered) 입력
 *                   /ultrasonic/front, /ultrasonic/down, /ultrasonic/left, /ultrasonic/right (sensor_msgs/msg/Range)
 *   2. 4채널 Cytron MD30C PWM/DIR 모터 제어 (20kHz, 10-bit LEDC) + 스톨 보호
 *   3. 4채널 ESP32 하드웨어 PCNT 4체배 직교 엔코더 카운팅
 *   4. 50Hz 메카넘 역기구학 + PID 속도 폐루프 제어 + 0.5초 Watchdog 비상정지
 *   5. 4채널 초음파(전방/하방/좌측/우측) 비차단 인터럽트 동시 측정 (TRIG 2그룹 교차 발사)
 *
 * [주의] UART0(GPIO1/3)은 micro-ROS USB 통신에 사용되므로 Serial.print() 디버깅 출력 금지!
 */

#include <Arduino.h>
#include <math.h>
#include <driver/pcnt.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/range.h>

// ==========================================
// 1. 하드웨어 핀맵 설정 (esp32_pinmap_and_hall_sensor_check.md 일치)
// ==========================================
// FL (앞왼쪽, Front-Left)
#define PIN_PWM_FL    25
#define PIN_DIR_FL    4
#define PIN_ENC_A_FL  34  // 입력 전용 (실측: 3.3V 동작 & 내장 풀업 확인 -> 저항 없이 직결)
#define PIN_ENC_B_FL  18

// FR (앞오른쪽, Front-Right)
#define PIN_PWM_FR    26
#define PIN_DIR_FR    16
#define PIN_ENC_A_FR  35  // 입력 전용 (실측: 3.3V 동작 & 내장 풀업 확인 -> 저항 없이 직결)
#define PIN_ENC_B_FR  19

// RL (뒤왼쪽, Rear-Left)
#define PIN_PWM_RL    27
#define PIN_DIR_RL    17
#define PIN_ENC_A_RL  36  // VP, 입력 전용 (실측: 3.3V 동작 & 내장 풀업 확인 -> 저항 없이 직결)
#define PIN_ENC_B_RL  13

// RR (뒤오른쪽, Rear-Right)
#define PIN_PWM_RR    33
#define PIN_DIR_RR    32
#define PIN_ENC_A_RR  39  // VN, 입력 전용 (실측: 3.3V 동작 & 내장 풀업 확인 -> 저항 없이 직결)
#define PIN_ENC_B_RR  14

// 초음파 센서 (HC-SR04 계열 ×4: 전방, 하방, 좌측, 우측)
#define PIN_TRIG_A      23  // 전방 + 하방 공유
#define PIN_TRIG_B      22  // 좌측 + 우측 공유
#define PIN_ECHO_FRONT  21  // 5V -> 3.3V 분압
#define PIN_ECHO_DOWN   12  // 5V -> 3.3V 분압 (부팅 시 LOW 유지 필수!)
#define PIN_ECHO_LEFT   15  // 5V -> 3.3V 분압
#define PIN_ECHO_RIGHT  5   // 5V -> 3.3V 분압

// 상태 표시 LED (micro-ROS 연결 상태)
#define PIN_STATUS_LED  2

// PWM 설정 (MD30C: 20kHz 이하, 10bit 0~1023)
#define PWM_FREQ      20000
#define PWM_RES       10
#define PWM_MAX_DUTY  1023
#define PWM_DEADZONE  40

enum WheelIdx { FL = 0, FR = 1, RL = 2, RR = 3, NUM_WHEELS = 4 };

const int PWM_PINS[NUM_WHEELS]   = { PIN_PWM_FL, PIN_PWM_FR, PIN_PWM_RL, PIN_PWM_RR };
const int DIR_PINS[NUM_WHEELS]   = { PIN_DIR_FL, PIN_DIR_FR, PIN_DIR_RL, PIN_DIR_RR };
const int ENC_A_PINS[NUM_WHEELS] = { PIN_ENC_A_FL, PIN_ENC_A_FR, PIN_ENC_A_RL, PIN_ENC_A_RR };
const int ENC_B_PINS[NUM_WHEELS] = { PIN_ENC_B_FL, PIN_ENC_B_FR, PIN_ENC_B_RL, PIN_ENC_B_RR };
const pcnt_unit_t PCNT_UNITS[NUM_WHEELS] = { PCNT_UNIT_0, PCNT_UNIT_1, PCNT_UNIT_2, PCNT_UNIT_3 };

// ==========================================
// 2. 로봇 및 제어 파라미터
// ==========================================
const float WHEEL_RADIUS   = 0.05f;   // 바퀴 반지름 (m) - 실제 섀시에 맞게 조정
const float LX             = 0.18f;   // 중심에서 전후 바퀴 거리 (m)
const float LY             = 0.18f;   // 중심에서 좌우 바퀴 거리 (m)
const float L_SUM          = LX + LY; // 0.36m

// PG42-4266-1270E 엔코더 (감속비 1/49, PCNT 4체배)
// 13PPR 기준: 13 * 4 * 49 = 2548 (7PPR일 경우 1372로 수정)
const float COUNTS_PER_REV = 13.0f * 4.0f * 49.0f;
const float RAD_PER_COUNT  = (2.0f * (float)M_PI) / COUNTS_PER_REV;

// PID 속도 제어 게인 (각속도 rad/s 기준)
const float KP = 35.0f;
const float KI = 80.0f;
const float KD = 0.5f;
const float KFF = 50.0f;              // 피드포워드 게인 (rad/s -> PWM duty)
const float INTEGRAL_MAX = 400.0f;

// 안전 및 타이밍 파라미터
const unsigned long CMD_TIMEOUT_MS     = 500;  // 0.5초 cmd_vel 미수신 시 정지
const unsigned long CONTROL_PERIOD_MS  = 20;   // 50Hz 제어 및 오도메트리
const unsigned long US_PERIOD_MS       = 30;   // 30ms마다 그룹 A/B 교차 측정
const int STALL_PWM_THRESHOLD          = 600;  // MD30C 스톨 보호 임계 PWM (~60%)
const unsigned long STALL_TIMEOUT_MS   = 1000; // 1초 이상 구속 시 출력 제한

// ==========================================
// 3. 글로벌 상태 변수
// ==========================================
// 목표 속도 (/cmd_vel)
volatile float target_vx = 0.0f;
volatile float target_vy = 0.0f;
volatile float target_wz = 0.0f;
volatile unsigned long last_cmd_time_ms = 0;

// 엔코더 & PID 상태
int16_t prev_pcnt[NUM_WHEELS]    = {0, 0, 0, 0};
float pid_integral[NUM_WHEELS]   = {0.0f, 0.0f, 0.0f, 0.0f};
float pid_prev_err[NUM_WHEELS]   = {0.0f, 0.0f, 0.0f, 0.0f};
unsigned long stall_timer[NUM_WHEELS] = {0, 0, 0, 0};

// 오도메트리 적분 상태
float odom_x   = 0.0f;
float odom_y   = 0.0f;
float odom_yaw = 0.0f;

// 초음파 인터럽트 측정 변수 (0: Front, 1: Down, 2: Left, 3: Right)
enum UsIdx { US_FRONT = 0, US_DOWN = 1, US_LEFT = 2, US_RIGHT = 3, NUM_US = 4 };
const int ECHO_PINS[NUM_US] = { PIN_ECHO_FRONT, PIN_ECHO_DOWN, PIN_ECHO_LEFT, PIN_ECHO_RIGHT };
volatile unsigned long echo_rise_us[NUM_US] = {0, 0, 0, 0};
volatile unsigned long echo_dur_us[NUM_US]  = {0, 0, 0, 0};
volatile bool echo_ready[NUM_US]            = {false, false, false, false};

// micro-ROS 객체
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;

rcl_publisher_t odom_pub;
nav_msgs__msg__Odometry odom_msg;

rcl_publisher_t us_pubs[NUM_US];
sensor_msgs__msg__Range us_msgs[NUM_US];

enum AgentState {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};
AgentState agent_state = WAITING_AGENT;

// 정적 문자열 버퍼 (micro-ROS 메모리 누수 방지)
char frame_odom[]       = "odom";
char frame_base[]       = "base_link";
char frame_us_front[]   = "ultrasonic_front_link";
char frame_us_down[]    = "ultrasonic_down_link";
char frame_us_left[]    = "ultrasonic_left_link";
char frame_us_right[]   = "ultrasonic_right_link";
char* us_frame_ids[NUM_US] = { frame_us_front, frame_us_down, frame_us_left, frame_us_right };
const char* us_topic_names[NUM_US] = {
  "/ultrasonic/front",
  "/ultrasonic/down",
  "/ultrasonic/left",
  "/ultrasonic/right"
};

// ==========================================
// 4. 초음파 ECHO 인터럽트 핸들러 (비차단 동시 측정)
// ==========================================
void IRAM_ATTR handleEchoISR(int idx) {
  if (digitalRead(ECHO_PINS[idx]) == HIGH) {
    echo_rise_us[idx] = micros();
  } else {
    unsigned long rise = echo_rise_us[idx];
    if (rise > 0) {
      echo_dur_us[idx] = micros() - rise;
      echo_ready[idx] = true;
      echo_rise_us[idx] = 0;
    }
  }
}

void IRAM_ATTR isr_echo_front() { handleEchoISR(US_FRONT); }
void IRAM_ATTR isr_echo_down()  { handleEchoISR(US_DOWN); }
void IRAM_ATTR isr_echo_left()  { handleEchoISR(US_LEFT); }
void IRAM_ATTR isr_echo_right() { handleEchoISR(US_RIGHT); }

// ==========================================
// 5. 하드웨어 PCNT 4체배 엔코더 초기화 및 읽기
// ==========================================
void setupPCNT() {
  for (int i = 0; i < NUM_WHEELS; i++) {
    // 실측 결과 3.3V에서 0~3.29V로 정상 스위칭(내장 풀업/푸시풀) 확인되어 INPUT 모드 사용
    pinMode(ENC_A_PINS[i], INPUT);
    pinMode(ENC_B_PINS[i], INPUT);

    pcnt_unit_t unit = PCNT_UNITS[i];

    // 채널 0: A상 엣지, B상 레벨 기준 방향 판별
    pcnt_config_t cfg0 = {};
    cfg0.pulse_gpio_num = ENC_A_PINS[i];
    cfg0.ctrl_gpio_num  = ENC_B_PINS[i];
    cfg0.channel        = PCNT_CHANNEL_0;
    cfg0.unit           = unit;
    cfg0.pos_mode       = PCNT_COUNT_DEC;
    cfg0.neg_mode       = PCNT_COUNT_INC;
    cfg0.lctrl_mode     = PCNT_MODE_REVERSE;
    cfg0.hctrl_mode     = PCNT_MODE_KEEP;
    cfg0.counter_h_lim  = 32767;
    cfg0.counter_l_lim  = -32768;
    pcnt_unit_config(&cfg0);

    // 채널 1: B상 엣지, A상 레벨 기준 방향 판별 (총 4체배 완성)
    pcnt_config_t cfg1 = {};
    cfg1.pulse_gpio_num = ENC_B_PINS[i];
    cfg1.ctrl_gpio_num  = ENC_A_PINS[i];
    cfg1.channel        = PCNT_CHANNEL_1;
    cfg1.unit           = unit;
    cfg1.pos_mode       = PCNT_COUNT_INC;
    cfg1.neg_mode       = PCNT_COUNT_DEC;
    cfg1.lctrl_mode     = PCNT_MODE_REVERSE;
    cfg1.hctrl_mode     = PCNT_MODE_KEEP;
    cfg1.counter_h_lim  = 32767;
    cfg1.counter_l_lim  = -32768;
    pcnt_unit_config(&cfg1);

    // 노이즈 글리치 필터 설정 (1us 이하 노이즈 무시)
    pcnt_set_filter_value(unit, 80);
    pcnt_filter_enable(unit);

    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
  }
}

int16_t readEncoderDelta(int idx) {
  int16_t count = 0;
  pcnt_get_counter_value(PCNT_UNITS[idx], &count);
  int16_t delta = (int16_t)(count - prev_pcnt[idx]); // 16-bit 오버플로 자동 보정
  prev_pcnt[idx] = count;
  return delta;
}

// ==========================================
// 6. 모터 PWM 제어 및 정지
// ==========================================
void setMotorDuty(int idx, int signed_duty) {
  bool forward = (signed_duty >= 0);
  int duty = abs(signed_duty);

  if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;
  if (duty > 0 && duty < PWM_DEADZONE) duty = PWM_DEADZONE;

  digitalWrite(DIR_PINS[idx], forward ? HIGH : LOW);
  ledcWrite(idx, duty);
}

void stopAllMotors() {
  for (int i = 0; i < NUM_WHEELS; i++) {
    ledcWrite(i, 0);
    pid_integral[i] = 0.0f;
    pid_prev_err[i] = 0.0f;
    stall_timer[i]  = 0;
  }
}

// ==========================================
// 7. micro-ROS 콜백 및 제어/오도메트리 루프
// ==========================================
void cmdVelCallback(const void* msgin) {
  const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msgin;
  target_vx = (float)msg->linear.x;
  target_vy = (float)msg->linear.y;
  target_wz = (float)msg->angular.z;
  last_cmd_time_ms = millis();
}

void fillStamp(builtin_interfaces__msg__Time* stamp) {
  int64_t time_ms = rmw_uros_epoch_millis();
  if (time_ms <= 0) {
    unsigned long now_ms = millis();
    stamp->sec = (int32_t)(now_ms / 1000);
    stamp->nanosec = (uint32_t)((now_ms % 1000) * 1000000UL);
  } else {
    stamp->sec = (int32_t)(time_ms / 1000);
    stamp->nanosec = (uint32_t)((time_ms % 1000) * 1000000ULL);
  }
}

void runControlAndOdom(float dt) {
  // 1) 엔코더 델타 및 실제 바퀴 각속도(rad/s) 측정
  float w_meas[NUM_WHEELS];
  int16_t deltas[NUM_WHEELS];
  for (int i = 0; i < NUM_WHEELS; i++) {
    deltas[i] = readEncoderDelta(i);
    w_meas[i] = ((float)deltas[i] * RAD_PER_COUNT) / dt;
  }

  // 2) 정기구학으로 실제 로봇 속도 계산 및 오도메트리 적분
  float vx_meas = (WHEEL_RADIUS / 4.0f) * (w_meas[FL] + w_meas[FR] + w_meas[RL] + w_meas[RR]);
  float vy_meas = (WHEEL_RADIUS / 4.0f) * (-w_meas[FL] + w_meas[FR] + w_meas[RL] - w_meas[RR]);
  float wz_meas = (WHEEL_RADIUS / (4.0f * L_SUM)) * (-w_meas[FL] + w_meas[FR] - w_meas[RL] + w_meas[RR]);

  odom_x   += (vx_meas * cosf(odom_yaw) - vy_meas * sinf(odom_yaw)) * dt;
  odom_y   += (vx_meas * sinf(odom_yaw) + vy_meas * cosf(odom_yaw)) * dt;
  odom_yaw += wz_meas * dt;

  // 3) Watchdog 체크 (0.5초 이상 명령 없으면 정지)
  bool timed_out = (millis() - last_cmd_time_ms > CMD_TIMEOUT_MS);
  if (timed_out) {
    stopAllMotors();
  } else {
    // 역기구학: 목표 바퀴 각속도 (rad/s)
    float w_target[NUM_WHEELS];
    w_target[FL] = (target_vx - target_vy - (L_SUM * target_wz)) / WHEEL_RADIUS;
    w_target[FR] = (target_vx + target_vy + (L_SUM * target_wz)) / WHEEL_RADIUS;
    w_target[RL] = (target_vx + target_vy - (L_SUM * target_wz)) / WHEEL_RADIUS;
    w_target[RR] = (target_vx - target_vy + (L_SUM * target_wz)) / WHEEL_RADIUS;

    bool all_zero = (fabsf(target_vx) < 1e-4f && fabsf(target_vy) < 1e-4f && fabsf(target_wz) < 1e-4f);
    if (all_zero) {
      stopAllMotors();
    } else {
      unsigned long now_ms = millis();
      for (int i = 0; i < NUM_WHEELS; i++) {
        float err = w_target[i] - w_meas[i];
        pid_integral[i] += err * dt;
        if (pid_integral[i] > INTEGRAL_MAX)  pid_integral[i] = INTEGRAL_MAX;
        if (pid_integral[i] < -INTEGRAL_MAX) pid_integral[i] = -INTEGRAL_MAX;

        float deriv = (err - pid_prev_err[i]) / dt;
        pid_prev_err[i] = err;

        float out = (KFF * w_target[i]) + (KP * err) + (KI * pid_integral[i]) + (KD * deriv);
        int duty = (int)out;

        // MD30C 스톨 보호: 고출력인데 엔코더 변화가 거의 없으면 출력 제한
        if (abs(duty) > STALL_PWM_THRESHOLD && abs(deltas[i]) < 2) {
          if (stall_timer[i] == 0) {
            stall_timer[i] = now_ms;
          } else if (now_ms - stall_timer[i] > STALL_TIMEOUT_MS) {
            duty = (duty > 0) ? 250 : -250; // 연속 정격 보호용 저출력 제한
          }
        } else {
          stall_timer[i] = 0;
        }

        setMotorDuty(i, duty);
      }
    }
  }

  // 4) /wheel/odom 발행
  fillStamp(&odom_msg.header.stamp);
  odom_msg.pose.pose.position.x = odom_x;
  odom_msg.pose.pose.position.y = odom_y;
  odom_msg.pose.pose.position.z = 0.0;

  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = sinf(odom_yaw * 0.5f);
  odom_msg.pose.pose.orientation.w = cosf(odom_yaw * 0.5f);

  odom_msg.twist.twist.linear.x  = vx_meas;
  odom_msg.twist.twist.linear.y  = vy_meas;
  odom_msg.twist.twist.angular.z = wz_meas;

  (void)rcl_publish(&odom_pub, &odom_msg, NULL);
}

// ==========================================
// 8. 초음파 그룹 교차 트리거 및 발행
// ==========================================
void triggerUltrasonicGroup(int trigPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
}

void publishUltrasonicIfReady(int idx) {
  if (!echo_ready[idx]) return;

  noInterrupts();
  unsigned long dur = echo_dur_us[idx];
  echo_ready[idx] = false;
  interrupts();

  // HC-SR04 범위: ~2cm (116us) ~ 400cm (23200us)
  float dist_m = (float)dur * 0.0001715f; // (dur * 343m/s) / 2 / 1e6
  if (dist_m < us_msgs[idx].min_range || dist_m > us_msgs[idx].max_range) {
    dist_m = INFINITY;
  }

  fillStamp(&us_msgs[idx].header.stamp);
  us_msgs[idx].range = dist_m;
  (void)rcl_publish(&us_pubs[idx], &us_msgs[idx], NULL);
}

// ==========================================
// 9. micro-ROS 엔티티 생성 및 해제
// ==========================================
bool createEntities() {
  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "esp32_mecanum_node", "", &support) != RCL_RET_OK) return false;

  // /cmd_vel 구독
  if (rclc_subscription_init_default(
        &cmd_vel_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel") != RCL_RET_OK) return false;

  // /wheel/odom 발행
  if (rclc_publisher_init_default(
        &odom_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/wheel/odom") != RCL_RET_OK) return false;

  odom_msg.header.frame_id.data     = frame_odom;
  odom_msg.header.frame_id.size     = strlen(frame_odom);
  odom_msg.header.frame_id.capacity = sizeof(frame_odom);
  odom_msg.child_frame_id.data      = frame_base;
  odom_msg.child_frame_id.size      = strlen(frame_base);
  odom_msg.child_frame_id.capacity  = sizeof(frame_base);

  // 초음파 4개 발행
  for (int i = 0; i < NUM_US; i++) {
    if (rclc_publisher_init_default(
          &us_pubs[i], &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
          us_topic_names[i]) != RCL_RET_OK) return false;

    us_msgs[i].header.frame_id.data     = us_frame_ids[i];
    us_msgs[i].header.frame_id.size     = strlen(us_frame_ids[i]);
    us_msgs[i].header.frame_id.capacity = strlen(us_frame_ids[i]) + 1;
    us_msgs[i].radiation_type           = sensor_msgs__msg__Range__ULTRASOUND;
    us_msgs[i].field_of_view            = 0.26f; // ~15도
    us_msgs[i].min_range                = 0.02f; // 2cm
    us_msgs[i].max_range                = 4.00f; // 400cm
  }

  // Executor 초기화 (구독 1개)
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(
        &executor, &cmd_vel_sub, &cmd_vel_msg,
        &cmdVelCallback, ON_NEW_DATA) != RCL_RET_OK) return false;

  // Jetson과 시간 동기화 시도 (최대 1초)
  rmw_uros_sync_session(1000);
  last_cmd_time_ms = millis();
  return true;
}

void destroyEntities() {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  (void)rcl_subscription_fini(&cmd_vel_sub, &node);
  (void)rcl_publisher_fini(&odom_pub, &node);
  for (int i = 0; i < NUM_US; i++) {
    (void)rcl_publisher_fini(&us_pubs[i], &node);
  }
  (void)rclc_executor_fini(&executor);
  (void)rcl_node_fini(&node);
  (void)rclc_support_fini(&support);
}

// ==========================================
// 10. Setup & Main Loop
// ==========================================
void setup() {
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);

  // micro-ROS USB Serial 트랜스포트 초기화
  set_microros_transports();

  // 모터 DIR & PWM(LEDC) 초기화
  for (int i = 0; i < NUM_WHEELS; i++) {
    pinMode(DIR_PINS[i], OUTPUT);
    digitalWrite(DIR_PINS[i], LOW);
    ledcSetup(i, PWM_FREQ, PWM_RES);
    ledcAttachPin(PWM_PINS[i], i);
    ledcWrite(i, 0);
  }

  // 엔코더 PCNT 초기화
  setupPCNT();

  // 초음파 TRIG/ECHO 초기화
  pinMode(PIN_TRIG_A, OUTPUT);
  pinMode(PIN_TRIG_B, OUTPUT);
  digitalWrite(PIN_TRIG_A, LOW);
  digitalWrite(PIN_TRIG_B, LOW);

  for (int i = 0; i < NUM_US; i++) {
    pinMode(ECHO_PINS[i], INPUT); // 5V->3.3V 분압 저항이 GND 풀다운 역할 겸함
  }
  attachInterrupt(digitalPinToInterrupt(PIN_ECHO_FRONT), isr_echo_front, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ECHO_DOWN),  isr_echo_down,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ECHO_LEFT),  isr_echo_left,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ECHO_RIGHT), isr_echo_right, CHANGE);

  stopAllMotors();
}

void loop() {
  static unsigned long last_ctrl_ms = 0;
  static unsigned long last_us_ms   = 0;
  static unsigned long last_ping_ms = 0;
  static bool trigger_group_a       = true;

  unsigned long now_ms = millis();

  switch (agent_state) {
    case WAITING_AGENT:
      stopAllMotors();
      // 500ms마다 Agent 연결 확인 + LED 느린 점멸
      if (now_ms - last_ping_ms >= 500) {
        last_ping_ms = now_ms;
        digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
          agent_state = AGENT_AVAILABLE;
        }
      }
      break;

    case AGENT_AVAILABLE:
      if (createEntities()) {
        agent_state = AGENT_CONNECTED;
        digitalWrite(PIN_STATUS_LED, HIGH);
        last_ctrl_ms = now_ms;
        last_us_ms   = now_ms;
        last_ping_ms = now_ms;
      } else {
        destroyEntities();
        agent_state = WAITING_AGENT;
      }
      break;

    case AGENT_CONNECTED:
      // 1) micro-ROS 콜백 처리 (/cmd_vel 수신)
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2));

      // 2) 50Hz PID 제어 및 /wheel/odom 발행
      if (now_ms - last_ctrl_ms >= CONTROL_PERIOD_MS) {
        float dt = (float)(now_ms - last_ctrl_ms) * 0.001f;
        last_ctrl_ms = now_ms;
        runControlAndOdom(dt);
      }

      // 3) 30ms마다 초음파 그룹 A(전방+하방) / 그룹 B(좌측+우측) 교차 측정
      if (now_ms - last_us_ms >= US_PERIOD_MS) {
        last_us_ms = now_ms;
        if (trigger_group_a) {
          publishUltrasonicIfReady(US_LEFT);
          publishUltrasonicIfReady(US_RIGHT);
          triggerUltrasonicGroup(PIN_TRIG_A);
        } else {
          publishUltrasonicIfReady(US_FRONT);
          publishUltrasonicIfReady(US_DOWN);
          triggerUltrasonicGroup(PIN_TRIG_B);
        }
        trigger_group_a = !trigger_group_a;
      }

      // 4) 1초마다 Agent 생존 확인
      if (now_ms - last_ping_ms >= 1000) {
        last_ping_ms = now_ms;
        if (rmw_uros_ping_agent(50, 1) != RMW_RET_OK) {
          agent_state = AGENT_DISCONNECTED;
        }
      }
      break;

    case AGENT_DISCONNECTED:
      stopAllMotors();
      digitalWrite(PIN_STATUS_LED, LOW);
      destroyEntities();
      agent_state = WAITING_AGENT;
      break;
  }
}
