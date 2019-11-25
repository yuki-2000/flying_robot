#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <inttypes.h>

const int BAUDRATE = 115200;

// OLEDの色の定義
const uint16_t BLACK = 0x0000;
const uint16_t BLUE = 0x001F;
const uint16_t RED = 0xF800;
const uint16_t GREEN = 0x07E0;
const uint16_t CYAN = 0x07FF;
const uint16_t MAGENTA = 0xF81F;
const uint16_t YELLOW = 0xFFE0;
const uint16_t WHITE = 0xFFFF;

// PIN assign
const uint8_t PIN_LED_BUILTIN = 32;
const uint8_t PIN_SWITCH = 17;
const uint8_t PIN_SCLK_OLED = 14;  // SCLK
const uint8_t PIN_MOSI_OLED = 13;  // MOSI (Master Output Slave Input)
const uint8_t PIN_MISO_OLED = 12;  // (Master Input Slave Output)
const uint8_t PIN_CS_OLED = 15;
const uint8_t PIN_DC_OLED = 16;  // OLED DC(Data/Command)
const uint8_t PIN_RST_OLED = 4;  // OLED Reset
const uint8_t PIN_SDA = 21;
const uint8_t PIN_SCL = 22;
const uint8_t PIN_SERVO_RUDDER = 12;
const uint8_t PIN_SERVO_ELEVATOR = 2;

// シリアル通信のバッファサイズ
const size_t SERIAL_BUFFER_SIZE = 64;

// サーボトリム
const int SERVO_RUDDER_TRIM = 80;
const int SERVO_ELEVATOR_TRIM = 90;

// サーボのPWMチャンネル
const int SERVO_RUDDER_CH = 0;
const int SERVO_ELEVATOR_CH = 1;

// PWMの出力関係
const int PWM_TIMER_BIT = 16;  // タイマーの精度
const int PWM_FREQ = 50;       // サーボ信号の１サイクル　50Hz:20ms
const float SERVO_MIN_WIDTH_MS = 0.5;  // 0degのときのPWMの幅
const float SERVO_MAX_WIDTH_MS = 2.5;  // 180degのときのPWMの幅

// 1loopの待機時間
const uint32_t WAIT_TIME_MS = 20;

// EEPROMのサイズ
const size_t EEPROM_SIZE = 128;

// EEPROMに保存したデータのアドレス
const int ADDRESS_SAVED_DATA = 20;

// エレベーターとラダーのサーボモータのトリム
// (Outputで指定した角度の0度の位置)
const float TRIM_RUDDER = 90;
const float TRIM_ELEVATOR = 90;

#endif
