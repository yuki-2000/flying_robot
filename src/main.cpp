#include <Adafruit_GFX.h>
#include <Adafruit_SSD1331.h>
#include <MPU9250_asukiaaa.h>
#include <MadgwickAHRS.h>
#include <SD.h>
#include <SPI.h>
#include "actuator.hpp"
#include "arduino_utility.hpp"
#include "controller.hpp"
#include "design3.hpp"
#include "parameters.hpp"
#include "serial_parser.h"
#include "utility.hpp"

// 状態をLCDへ表示する関数
void display_task(void *);
// センサーの値を読み取りsensorsに代入する関数
void read_sensors(SensorInfo &sensors);
// センサーの値や出力値をシリアルに出力する関数
void display_data(const SensorInfo sensors, const OutputInfo outputs);
// コマンドをシリアルから読み込んで実行する関数
void execute_command();

SensorInfo sensors;
OutputInfo outputs;
MPU9250 mpu;
Madgwick filter;
Adafruit_SSD1331 oled = Adafruit_SSD1331(
    PIN_CS_OLED, PIN_DC_OLED, PIN_MOSI_OLED, PIN_SCLK_OLED, PIN_RST_OLED);
SavedData saved_data;
TaskHandle_t display_th;

void setup() {
    delay(500);

    // ピンセットアップ
    pinMode(PIN_LED_BUILTIN, OUTPUT);
    pinMode(PIN_SWITCH, INPUT_PULLUP);
    digitalWrite(PIN_LED_BUILTIN, LOW);

    // シリアル通信開始
    Serial.begin(BAUDRATE);

    // コア0にOLED表示タスク割り当て
    xTaskCreatePinnedToCore(display_task, "display_task", 8192, NULL, 5,
                            &display_th, 0);

    // I2Cインスタンス開始
    Wire.begin(PIN_SDA, PIN_SCL, 100000);
    Wire.setTimeOut(500);

    // バイアスの読み取り
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.commit();
    saved_data = read_from_eeprom<SavedData>(ADDRESS_SAVED_DATA);
    Serial.print(to_string(saved_data));

    // MPU9250インスタンス開始
    mpu.setWire(&Wire);
    mpu.beginAccel();
    mpu.beginMag();
    mpu.beginGyro();

    // サーボなどのアクチュエータの初期化
    actuator_init();

    // controller.cpp内の初期化
    controller_init();

    filter.begin(10);
}

void loop() {
    read_sensors(sensors);

    // サーボの角度を計算
    outputs = calc_output(sensors);

    // サーボを動かす
    move_actuator(outputs);

    // センサーの値や出力をプロットする
    display_data(sensors, outputs);

    // コマンドをシリアルから読み込んで実行する関数
    execute_command();

    delay(WAIT_TIME_MS);
}

void read_sensors(SensorInfo &sensors) {
    static uint32_t old_time_ms = 0;
    float t = 180 * TO_RAD;
    const float mat[3][3] = {
        {1, 0, 0},
        {0, cosf(t), -sinf(t)},
        {0, sinf(t), +cosf(t)},
    };

    sensors.time_ms = millis();
    sensors.dt_ms =
        old_time_ms == 0 ? WAIT_TIME_MS : sensors.time_ms - old_time_ms;
    old_time_ms = sensors.time_ms;

    // データ読み取り
    mpu.accelUpdate();
    mpu.gyroUpdate();
    mpu.magUpdate();

    // データ変数更新
    // ジャイロ・加速度に磁気の軸を合わせる
    sensors.accel[0] = mpu.accelX();
    sensors.accel[1] = mpu.accelY();
    sensors.accel[2] = mpu.accelZ();
    sensors.gyro[0] = -(mpu.gyroX() - saved_data.gyro[0]);
    sensors.gyro[1] = -(mpu.gyroY() - saved_data.gyro[1]);
    sensors.gyro[2] = -(mpu.gyroZ() - saved_data.gyro[2]);
    sensors.mag[0] = +(mpu.magY() - saved_data.mag[0]);
    sensors.mag[1] = +(mpu.magX() - saved_data.mag[1]);
    // TODO: check this sign
    sensors.mag[2] = -(mpu.magZ() - saved_data.mag[2]);

    // センサ全体を回転させる
    rotate_vector3(mat, sensors.accel);
    rotate_vector3(mat, sensors.gyro);
    rotate_vector3(mat, sensors.mag);

    // フィルタにサンプリングレートを設定し，姿勢を更新
    filter.begin(1000.0 / sensors.dt_ms);
    filter.update(sensors.gyro[0], sensors.gyro[1], sensors.gyro[2],
                  sensors.accel[0], sensors.accel[1], sensors.accel[2],
                  sensors.mag[0], sensors.mag[1], sensors.mag[2]);
    sensors.roll = filter.getRoll();
    sensors.pitch = filter.getPitch();
    sensors.yaw = filter.getYaw();

    // スイッチ読み取り
    sensors.sw = digitalRead(PIN_SWITCH) == LOW ? 1 : 0;
}

// コア0用のタスク関数
// OLED(SSD1331)にAHRSのデータを表示する
void display_task(void *pvParameters) {
    oled.begin();  // OLED通信開始

    // ラベル表示部分
    oled.fillScreen(BLACK);
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.setTextColor(WHITE);
    oled.println("accel gyro mag");
    oled.setTextColor(RED);
    oled.println("AX      MX ");
    oled.setTextColor(GREEN);
    oled.println("AY      MY ");
    oled.setTextColor(BLUE);
    oled.println("AZ      MZ ");
    oled.setTextColor(RED);
    oled.println("GX      RL");
    oled.setTextColor(GREEN);
    oled.println("GY      PT");
    oled.setTextColor(BLUE);
    oled.println("GZ      YW");
    oled.setTextColor(WHITE);

    // Loop
    while (true) {
        // 更新部分を塗りつぶす
        // oled.fillRect(6 * 2, 8 * 1, 6 * 6, 8 * 4, BLACK);
        // oled.fillRect(6 * 10, 8 * 1, 6 * 10, 8 * 4, BLACK);
        // oled.fillRect(6 * 2, 8 * 4, 6 * 10, 8 * 7, BLACK);
        oled.fillRect(6 * 2, 8 * 1, 6 * 6, 8 * 7, BLACK);
        oled.fillRect(6 * 10, 8 * 1, 6 * 10, 8 * 7, BLACK);

        // AHRSデータ表示
        for (int i = 0; i < 3; i++) {
            oled.setCursor(6 * 2, 8 * (i + 1));
            oled.print(sensors.accel[i]);
            oled.setCursor(6 * 2, 8 * (i + 4));
            oled.print(sensors.gyro[i]);
            oled.setCursor(6 * 10, 8 * (i + 1));
            oled.print(sensors.mag[i]);
        }
        oled.setCursor(6 * 10, 8 * 4);
        oled.print(sensors.roll);
        oled.setCursor(6 * 10, 8 * 5);
        oled.print(sensors.pitch);
        oled.setCursor(6 * 10, 8 * 6);
        oled.print(sensors.yaw);

        delay(WAIT_TIME_MS);
    }
}

// センサーの値や出力を表示する
void display_data(const SensorInfo sensors, const OutputInfo outputs) {
    Serial.print(to_string(sensors));
    Serial.println(to_string(outputs));
}

// コマンドをシリアルから読み込んで実行する関数
void execute_command() {
    static SerialParser parser;
    while (parser.read_line()) {
        Command command = parser.parse_command();
        Serial.printf("command = %d\n", (int)command);

        if (command == COMMAND_RESET) {
            reset_microcontroller();
        } else if (command == COMMAND_BIAS) {
            detect_bias(mpu, saved_data);

            EEPROM.commit();
            write_to_eeprom(ADDRESS_SAVED_DATA, saved_data);
            EEPROM.commit();

            reset_microcontroller();
        }
    }
}




// コマンドをシリアルから読み込んで実行する関数
void new_bias() {



            detect_bias(mpu, saved_data);

            EEPROM.commit();
            write_to_eeprom(ADDRESS_SAVED_DATA, saved_data);
            EEPROM.commit();

            reset_microcontroller();
        
    
}

