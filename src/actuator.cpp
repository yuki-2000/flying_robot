#include "actuator.hpp"
#include "parameters.hpp"

uint32_t servo_pwm_count(float v);

void actuator_init() {
    // SERVO_RUDDERのPWMの設定
    ledcSetup(SERVO_RUDDER_CH, PWM_FREQ, PWM_TIMER_BIT);  // チャンネルの設定
    ledcAttachPin(PIN_SERVO_RUDDER, SERVO_RUDDER_CH);  // ピンの登録
    ledcWrite(PIN_SERVO_RUDDER, servo_pwm_count(SERVO_RUDDER_TRIM));

    // SERVO_ELEVATORのPWMの設定
    ledcSetup(SERVO_ELEVATOR_CH, PWM_FREQ, PWM_TIMER_BIT);  // チャンネルの設定
    ledcAttachPin(PIN_SERVO_ELEVATOR, SERVO_ELEVATOR_CH);  // ピンの登録
    ledcWrite(PIN_SERVO_ELEVATOR, servo_pwm_count(SERVO_ELEVATOR_TRIM));
}

void servo_at_zero() {
    while (true) {
        ledcWrite(SERVO_RUDDER_CH, servo_pwm_count(SERVO_RUDDER_TRIM));
        ledcWrite(SERVO_ELEVATOR_CH, servo_pwm_count(SERVO_ELEVATOR_TRIM));
    }
}

void move_actuator(const OutputInfo& outputs) {
    float angle_rudder =
        TRIM_RUDDER + constrain(outputs.servo_angle_rudder, -30, 30);
    float angle_pitch =
        TRIM_ELEVATOR + constrain(outputs.servo_angle_elevator, -30, 30);
    // Serial.println(angle_pitch);
    // Serial.println(angle_rudder);
    // ロール制御用フィンの動作
    ledcWrite(SERVO_RUDDER_CH, servo_pwm_count(angle_rudder));
    ledcWrite(SERVO_ELEVATOR_CH, servo_pwm_count(angle_pitch));
    digitalWrite(PIN_LED_BUILTIN, outputs.led_on ? HIGH : LOW);
}

uint32_t servo_pwm_count(float v) {
    float vv = v / 180.0;
    uint32_t period_ms = (1000 / PWM_FREQ);  // 20 ms
    uint32_t max_value = (1L << PWM_TIMER_BIT);
    return (uint32_t)(
        (max_value / period_ms) *
        (SERVO_MIN_WIDTH_MS + vv * (SERVO_MAX_WIDTH_MS - SERVO_MIN_WIDTH_MS)));
}

// void check_servo_trim() {
//   char buff[32];
//   int index = 0;
//   bool flag = true;
//   float angle = 90;
//
//   while (flag) {
//     while (Serial.available()) {
//       char c = Serial.read();
//       buff[index] = c;
//       index++;
//       if (c == '\n') {
//         buff[index] = '\0';
//         angle = atof(buff);
//         index = 0;
//       }
//
//       if (angle > 360)
//         flag = false;
//     }
//     ledcWrite(LEDC_CHANNEL_A, servo_pwm_count(angle));
//     ledcWrite(LEDC_CHANNEL_B, servo_pwm_count(angle));
//     delay(100);
//   }
//   ResetMicrocontroller();
// }
