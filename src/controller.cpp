#include "controller.hpp"
#include "parameters.hpp"
#include    <math.h>


//初期化関連

// 変数の初期化
//xyzの順
float hight =1.0;
float velocity[3]= {0,0,0};
float position[3]= {0,0,hight};

//pitch,yaw,roll　1ステップ前の値を保存しておきたい
//errors0が今回の,errors1が一回前の差分
//pitch yaw rollの順
double errors0[3]= {0,0,0};
double errors1[3]= {0,0,0};

//積分ゲイン用
double integral[3]={0,0,0};

//pidそれぞれの出力
double p[3]={0,0,0};
double i[3]={0,0,0};
double d[3]={0,0,0};

//元がintなので0になって後々0除算になってしまう
//WAIT_TIME_MSは100
double wait_s = WAIT_TIME_MS/1000;




//ユーザー設定

//ゲイン　pitch,yaw,roll
//なおアクチュエータの出力は±30度
double KP[3]={2,2,2};
double KI[3]={2,2,2};
double KD[3]={0.02,0.02,0.02};

//目標値　pitch,yaw,roll
double target_values[3]= {0,0,0};




// controller.cpp内の初期化を行う関数
void controller_init() {}






// 制御量を計算する関数
// 0.1 s間隔で呼び出される
// この関数内ではdelayなど処理時間のかかる処理は行わない
OutputInfo calc_output(const SensorInfo& sensors) {
    OutputInfo outputs;

    //ボタンを押してリセット
    if (digitalRead(PIN_SWITCH) == false){
        outputs.led_on = true;
        integral[0]=0;
        integral[1]=0;
        integral[2]=0;
        target_values[0] = sensors.pitch;
        target_values[1] = sensors.yaw;
        target_values[2] = sensors.roll;
    }
    else{
        outputs.led_on = false;
    }


    // 経過時間[ms]を経過時間[s]に変換
    float seconds = sensors.time_ms / 1000;
    float g=1.05;


   //前の値を保存しておく
    errors1[0] = errors0[0];
    errors1[1] = errors0[1];
    errors1[2] = errors0[2];

    //現在の誤差を計算
    errors0[0] = target_values[0] - sensors.pitch;
    errors0[1] = target_values[1] - sensors.yaw;
    errors0[2] = target_values[2] - sensors.roll;


    //errorsの時間積分を計算
    //0.02はwait_MSのs表示、式中に変数で入れるとなぜか0になって0除算に
    integral[0] += ((errors0[0] + errors1[0]) / 2.0) * 0.02;
    integral[1] += ((errors0[1] + errors1[1]) / 2.0) * 0.02;
    integral[2] += ((errors0[2] + errors1[2]) / 2.0) * 0.02;


    //各出力　pitch,yaw,roll
    p[0] = KP[0] * errors0[0];
    p[1] = KP[1] * errors0[1];
    p[2] = KP[2] * errors0[2];

    i[0] = KI[0] * integral[0];
    i[1] = KI[1] * integral[1];
    i[2] = KI[2] * integral[2];

    d[0]= KD[0]*(errors0[0]-errors1[0])/0.02;
    d[1]= KD[1]*(errors0[1]-errors1[1])/0.02;
    d[2]= KD[2]*(errors0[2]-errors1[2])/0.02;




    // アクチュエータへの出力計算
    //なおアクチュエータの出力は±30度
    if (sensors.pitch >15){
        outputs.servo_angle_elevator = 30;
        outputs.servo_angle_rudder = 30;
    }else{
        outputs.servo_angle_elevator = 0;
        outputs.servo_angle_rudder = 0;
        //outputs.servo_angle_elevator = +{p[0] + i[0] + d[0]};
    }
    //ロールに応じて垂直尾翼を動かすことにする。
    //outputs.servo_angle_rudder = +{p[2] + i[2] + d[2]};

    






    //シリアルモニタへの出力

    char col[256];
    sprintf(col,"|  value  |  pitch  |   yaw   |   roll    |");
    Serial.println(col);  

    char posture[256];
    sprintf(posture,"| posture |  %5.2f  |  %5.2f  |  %5.2f  |", sensors.pitch, sensors.yaw,sensors.roll);
    Serial.println(posture);  

    char errors_ch[256];
    sprintf(errors_ch,"| errors  |  %5.2f  |  %5.2f  |  %5.2f  |", errors0[0], errors0[1],errors0[2]);
    Serial.println(errors_ch);  

    char p_out_ch[256];
    sprintf(p_out_ch,"|  p_out  |  %5.2f  |  %5.2f  |  %5.2f  |", p[0], p[1], p[2]);
    Serial.println(p_out_ch);  

    char output_ch[256];
    sprintf(output_ch,"| output  |  %5.2f  |  %5.2f  |  %5.2f  |", outputs.servo_angle_elevator, outputs.servo_angle_rudder, 00.00);
    Serial.println(output_ch);  


    

    Serial.println(WAIT_TIME_MS);
    Serial.println(wait_s);




    return outputs;

}


//なお，サンプルプログラムではデフォルトでセンサーの値と
//出力の値が表示されるように設定されています．
//これを消したい場合はmain.cpp/loop内の
//display_data(sensors, outputs);の行を
//コメントアウトしてください．
//これはアップデートする関数なのでディスプレイをoffにするものではない

//ディスプレイをoffにしたいときは、
// コア0用のタスク関数
// OLED(SSD1331)にAHRSのデータを表示する
//void display_task(void *pvParameters) {  内の
    //oled.begin();  // OLED通信開始
//というようにコメントアウトする