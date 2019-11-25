#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "design3.hpp"
#include "utility.hpp"

// controller.cpp内の初期化を行う関数
void controller_init();
// 制御量を計算する関数
OutputInfo calc_output(const SensorInfo& sensors);
#endif
