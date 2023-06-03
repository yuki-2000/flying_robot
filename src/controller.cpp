#include "controller.hpp"
#include "parameters.hpp"
#include    <math.h>

// controller.cpp内の初期化を行う関数
void controller_init() {


}




// 変数の初期化
//xyzの順
float hight =1.0;
float velocity[3]= {0,0,0};
float position[3]= {0,0,hight};
//pitch,yaw,roll　1ステップ前の値を保存しておきたい
//error0が一回前、error1が今回の角度差分
//pitch yaw rollの順
float error0[3]= {0,0,0};
float error1[3]= {0,0,0};
float integral[3]={0,0,0};

float KP[3]={10,10,10};
float KI[3]={10,10,10};
float KD[3]={10,10,10};

float p[3]={0,0,0};
float i[3]={0,0,0};
float d[3]={0,0,0};


// 制御量を計算する関数
// 0.1 s間隔で呼び出される
// この関数内ではdelayなど処理時間のかかる処理は行わない
OutputInfo calc_output(const SensorInfo& sensors) {
    OutputInfo outputs;




    // 経過時間[ms]を経過時間[s]に変換
    float seconds = sensors.time_ms / 1000;
    float g=1.05;

    //進行方向と機体向きの角度差をとりたい
    //進行方向の速度ベクトルを知りたいが、加速度ベクトルしかないので積分する
    velocity[0] += sensors.accel[0]* WAIT_TIME_MS/1000;
    velocity[1] += sensors.accel[1]* WAIT_TIME_MS/1000;
    velocity[2] += (sensors.accel[2]-g)* WAIT_TIME_MS/1000;

    Serial.println("velocitydelta");
    Serial.println(sensors.accel[0]* WAIT_TIME_MS/1000);
    Serial.println(sensors.accel[1]* WAIT_TIME_MS/1000);
    Serial.println(sensors.accel[2]* WAIT_TIME_MS/1000);
    Serial.println("velocitydelta");

    Serial.println(WAIT_TIME_MS);

    position[0] += velocity[0]* WAIT_TIME_MS/1000;
    position[1] += velocity[1]* WAIT_TIME_MS/1000;
    position[2] += velocity[2]* WAIT_TIME_MS/1000;

    //内積から角度を求める.ピッチはdgreeなので合わせる。
    float velo_pitch = acos(velocity[0]/sqrt(velocity[0]*velocity[0] + velocity[3]*velocity[3])) * 180/M_PI;
    float velo_yaw = acos(velocity[0]/sqrt(velocity[0]*velocity[0] + velocity[2]*velocity[2])) * 180/M_PI;
    float velo_roll = acos(velocity[1]/sqrt(velocity[1]*velocity[1] + velocity[2]*velocity[2])) * 180/M_PI;

    Serial.println(velo_pitch);
    Serial.println(velo_yaw);
    Serial.println(velo_roll);

   //前の値を保存しておく
    error0[0] = error1[0];
    error0[1] = error1[1];
    error0[2] = error1[2];

    //error1[0] = sensors.pitch + 5 - velo_pitch;
    //error1[1] = 0 - velo_yaw;
    //error1[2] = 0 - velo_roll;

    error1[0] = 5-sensors.pitch ;
    error1[1] = 0 - sensors.yaw;
    error1[2] = 0 - sensors.roll;
   

    integral[0] += (error1[0] + error0[0]) / 2.0 * WAIT_TIME_MS/1000;
    integral[1] += (error1[1] + error0[1]) / 2.0 * WAIT_TIME_MS/1000;
    integral[2] += (error1[2] + error0[2]) / 2.0 * WAIT_TIME_MS/1000;

    p[0] = KP[0] * error1[0];
    p[1] = KP[1] * error1[1];
    p[2] = KP[2] * error1[2];

    i[0] = KI[0] * integral[0];
    i[1] = KI[1] * integral[1];
    i[2] = KI[2] * integral[2];

    d[0]= KD[0]*(error1[0]-error1[0])*1000/WAIT_TIME_MS;
    d[1]= KD[1]*(error1[1]-error1[1])*1000/WAIT_TIME_MS;
    d[2]= KD[2]*(error1[2]-error1[2])*1000/WAIT_TIME_MS;

    if (sensors.pitch >15){
        outputs.servo_angle_elevator = 30;
        outputs.servo_angle_rudder = 30;
    }else{
        outputs.servo_angle_elevator = p[0] + i[0] + d[0];
        outputs.servo_angle_rudder = 0;
    }
    //outputs.servo_angle_rudder = p[1] + i[1] + d[1];
    
    Serial.println("PID");
    Serial.println(p[0]);
    Serial.println(i[0]);
    Serial.println(d[0]);
    
    return outputs;
}
