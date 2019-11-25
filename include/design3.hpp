#ifndef DESIGN3_H
#define DESIGN3_H

#include <Arduino.h>
#include "EEPROM.h"
#include "MPU9250_asukiaaa.h"
#include "utility.hpp"

// センサーの値をまとめるSensorInfo構造体
typedef struct _SensorInfo {
    uint32_t time_ms;  // 時間[ms]
    uint32_t dt_ms;    // 時間差[ms]
    float accel[3];    // 加速度ベクトル[g]
    float gyro[3];     // 角速度ベクトル[dps]
    float mag[3];      // 磁気ベクトル[Gauss]
    float roll;        // ロール角[deg]
    float pitch;       // ピッチ角[deg]
    float yaw;         // ヨー角[deg]
    uint16_t sw;       // スイッチ(0か1か)
    uint16_t cmd;      // コマンド
} SensorInfo;

// 出力をまとめるOutputInfo構造体
typedef struct _OutputInfo {
    float servo_angle_rudder;
    float servo_angle_elevator;
    bool led_on;
} OutputInfo;

// 磁気，ジャイロのバイアスを格納する構造体
typedef struct {
    float gyro[3];
    float mag[3];
} SavedData;

// マイコンをリセットする関数
void reset_microcontroller();

// センサーの値をStringに変換する関数
String to_string(const SensorInfo& sensors);

// 出力の値をStringに変換する関数
String to_string(const OutputInfo& outputs);

// SavedDataをStringに変換する関数
String to_string(const SavedData& saved_data);

// 3次元ベクトルvecをmat
void rotate_vector3(const float mat[3][3], float vec[3]);

// 加速度のノルムを計算する関数
float calc_accel_norm(const SensorInfo& sensors);

// コマンドのフラグ
typedef enum _Command {
    COMMAND_NONE,
    // COMMAND_CALIBRATION,
    COMMAND_RESET,
    COMMAND_BIAS,
    // COMMAND_ZERO,
} Command;

void detect_bias(MPU9250& mpu, SavedData& saved_data);
void detect_gyro_bias(SavedData& bias, MPU9250& mpu);
void detect_mag_bias(SavedData& bias, MPU9250& mpu);

#endif
