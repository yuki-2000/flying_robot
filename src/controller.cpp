#include "controller.hpp"
#include "parameters.hpp"

// controller.cpp内の初期化を行う関数
void controller_init() {
    // 変数の初期化
}

// 制御量を計算する関数
// 0.1 s間隔で呼び出される
// この関数内ではdelayなど処理時間のかかる処理は行わない
OutputInfo calc_output(const SensorInfo& sensors) {
    OutputInfo outputs;
    // 経過時間[ms]を経過時間[s]に変換
    int seconds = sensors.time_ms / 1000;

    if (seconds % 2 == 0) {
        // エレベーターとラダーの角度
        outputs.servo_angle_elevator = 0;
        outputs.servo_angle_rudder = 0;

        // 秒数が偶数ならLEDを点灯させる
        outputs.led_on = true;
    } else {
        // エレベーターとラダーの角度
        outputs.servo_angle_elevator = 10;
        outputs.servo_angle_rudder = 10;

        // 秒数が奇数ならLEDを消灯させる
        outputs.led_on = false;
    }
    return outputs;
}