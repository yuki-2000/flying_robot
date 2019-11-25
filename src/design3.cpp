#include "design3.hpp"

// マイコンをリセットする関数
void reset_microcontroller() { ESP.restart(); }

// センサーの値をStringに変換する関数
String to_string(const SensorInfo &sensors) {
    char buff[256];

    sprintf(buff,
            "time_ms: %d\n"
            "dt_ms: %d\n"
            "ax: %lf, ay: %lf, az:%lf\n"
            "gx: %lf, gy: %lf, gz:%lf\n"
            "mx: %lf, my: %lf, mz:%lf\n"
            "roll: %lf, pitch: %lf, yaw:%lf\n"
            "switch:%d,cmd: %d\n",
            sensors.time_ms,                                       //
            sensors.dt_ms,                                         //
            sensors.accel[0], sensors.accel[1], sensors.accel[2],  //
            sensors.gyro[0], sensors.gyro[1], sensors.gyro[2],     //
            sensors.mag[0], sensors.mag[1], sensors.mag[2],        //
            sensors.roll, sensors.pitch, sensors.yaw,              //
            sensors.sw, sensors.cmd);

    return String(buff);
}

// 出力の値をStringに変換する関数
String to_string(const OutputInfo &outputs) {
    char buff[256];

    sprintf(buff,
            "servo_angle_elevator:%lf\n"
            "servo_angle_rudder:%lf\n"
            "led_on:%d",
            outputs.servo_angle_elevator, outputs.servo_angle_rudder,
            outputs.led_on ? 1 : 0);

    return String(buff);
}

// SavedDataをStringに変換する関数
String to_string(const SavedData &saved_data) {
    char buff[256];

    sprintf(buff,
            "gx:%+08.3f, gy:%+08.3f, gz:%+08.3f\n"  // gyro
            "mx:%+6.3f, my:%+6.3f, mz:%+6.3f\n"     // mag
            ,
            saved_data.gyro[0], saved_data.gyro[1], saved_data.gyro[2],
            saved_data.mag[0], saved_data.mag[1], saved_data.mag[2]);

    return String(buff);
}

// 3次元ベクトルvecをmat
void rotate_vector3(const float mat[3][3], float vec[3]) {
    float temp[3];

    for (int i = 0; i < 3; i++) {
        temp[i] = 0;
        for (int j = 0; j < 3; j++) {
            temp[i] += mat[i][j] * vec[j];
        }
    }

    memcpy((void *)vec, (void *)temp, 3 * sizeof(float));
}

// 加速度のノルムを計算する関数
float calc_accel_norm(const SensorInfo &sensors) {
    float ret = 0;
    for (int i = 0; i < 3; i++) ret += sensors.accel[i] * sensors.accel[i];

    return sqrt(ret);
}

// バイアス推定してEEPROMへ書き込む関数
void detect_bias(MPU9250 &mpu, SavedData &bias) {
    detect_gyro_bias(bias, mpu);
    detect_mag_bias(bias, mpu);
}

// ジャイロバイアス推定機
void detect_gyro_bias(SavedData &bias, MPU9250 &mpu) {
    Serial.print("Detect gyro bias...\r\n");
    Serial.print("Don't move sensor.\r\n");
    Serial.print("Detecting will start after 3 sec.\r\n");
    delay(3000);

    float sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    float gx, gy, gz;

    for (int i = 0; i < 100; i++) {
        mpu.gyroUpdate();
        gx = mpu.gyroX();
        gy = mpu.gyroY();
        gz = mpu.gyroZ();
        sum_x += gx;
        sum_y += gy;
        sum_z += gz;

        if (i % 10 == 9) {
            Serial.print(i + 1);
            Serial.print("percent fin.\r\n");
        }
        delay(100);
    }

    gx = sum_x / 100.0f;
    gy = sum_y / 100.0f;
    gz = sum_z / 100.0f;

    Serial.print("(gx,gy,gz)=");
    Serial.print(gx);
    Serial.print(",");
    Serial.print(gy);
    Serial.print(",");
    Serial.print(gz);
    Serial.print("\r\n");

    bias.gyro[0] = gx;
    bias.gyro[1] = gy;
    bias.gyro[2] = gz;
}

// 磁気バイアス推定機
void detect_mag_bias(SavedData &bias, MPU9250 &mpu) {
    Serial.print("Detect mag bias...\r\n");
    Serial.print("Move sensor.\r\n");
    Serial.print("Detecting will start after 3 sec.\r\n");
    delay(3000);

    float a, b, c, r;
    double output[4];
    float mx, my, mz;
    double A[4][4] = {0};
    double B[4] = {0};

    for (int i = 0; i < 100; i++) {
        // センサーの値を読み取る
        mpu.magUpdate();
        mx = mpu.magX();
        my = mpu.magY();
        mz = mpu.magZ();

        // 左辺(係数行列)
        A[0][0] += 2.0f * mx * mx;
        A[0][1] += 2.0f * mx * my;
        A[0][2] += 2.0f * mx * mz;
        A[0][3] += 2.0f * mx;

        A[1][0] += 2.0f * my * mx;
        A[1][1] += 2.0f * my * my;
        A[1][2] += 2.0f * my * mz;
        A[1][3] += 2.0f * my;

        A[2][0] += 2.0f * mz * mx;
        A[2][1] += 2.0f * mz * my;
        A[2][2] += 2.0f * mz * mz;
        A[2][3] += 2.0f * mz;

        A[3][0] += 2.0f * mx;
        A[3][1] += 2.0f * my;
        A[3][2] += 2.0f * mz;
        A[3][3] += 2.0f * 1.0f;

        // 右辺
        B[0] += mx * (mx * mx + my * my + mz * mz);
        B[1] += my * (mx * mx + my * my + mz * mz);
        B[2] += mz * (mx * mx + my * my + mz * mz);
        B[3] += mx * mx + my * my + mz * mz;

        if (i % 10 == 9) {
            Serial.print(i + 1);
            Serial.println(" percent fin.");
        }
        delay(100);
    }
    solve<4>(A, output, B);  // 連立方程式を計算

    // a,b,c,dのパラメータがB[0],B[1],B[2],B[3]に代入されているので
    // a,b,c,rを代入して関数を終わる
    a = output[0];
    b = output[1];
    c = output[2];
    r = sqrt(a * a + b * b + c * c + 2 * output[3]);

    Serial.print("(mbx,mby,mbz,r)=");
    Serial.print(a);
    Serial.print(",");
    Serial.print(b);
    Serial.print(",");
    Serial.print(c);
    Serial.print(",");
    Serial.print(r);
    Serial.print("\r\n");

    bias.mag[0] = a;
    bias.mag[1] = b;
    bias.mag[2] = c;
}
