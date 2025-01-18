// プログラム作成：九工大大竹研
//参考：2サーボ駆動型羽ばたき機のArduino CODE（2　Servo）http://kakutaclinic.life.coocan.jp/SFOsys2S.html
#include<ESP32Servo.h>  // https://github.com/madhephaestus/ESP32Servo
#include "sbus.h"       // https://github.com/bolderflight/sbus
#include <Ticker.h>
#define PI 3.141592653589793

bfs::SbusRx sbus_rx(&Serial1,36,18,true);  //Rx:36, Tx:18 反転あり
bfs::SbusData data;

Ticker timer_sin;
Ticker timer_servo;
volatile int flag_square_wave = 0;    //矩形波の場合は1，Sin波の場合は1以外．


/////// 設定 /////////////////////////////////////////////////////////
const int dt_sin = 3;              //sin波生成の周期 ms   dt_sin <= dt_servo にする
const int dt_servo = 4;           //サーボの命令更新周期 ms　使用するサーボに合わせる　標準は20（50Hz）
const int servo_deg_neutral = 90;  //サーボの中心角度 deg単位　使用するサーボに合わせる
const int servo_deg_min = 30;      //サーボの最小角度 deg単位　使用するサーボに合わせる
const int servo_deg_max = 150;     //サーボの最大角度 deg単位　使用するサーボに合わせる
const int servo_us_neutral = 1500; //サーボの中心角度 マイクロ秒単位　使用するサーボに合わせる
const int servo_us_min = 900;      //サーボの最小角度 マイクロ秒単位　使用するサーボに合わせる
const int servo_us_max = 2100;     //サーボの最大角度 マイクロ秒単位　使用するサーボに合わせる
const int propo_neutral = 1024;    //プロポの中間値　Serial.printで確認する
const int propo_min = 352;         //プロポの最小値　Serial.printで確認する
const int propo_max = 1696;        //プロポの最大値　Serial.printで確認する
const int servo_left_pin = 26;   //左サーボのピン番号
const int servo_right_pin = 19;  //右サーボのピン番号
////////////////////////////////////////////////////////////////////

const int servo_period = 1000/dt_servo; //サーボの命令更新周波数 Hz
const int servo_0deg_us =map( 0,0,servo_deg_max-servo_deg_min,0,servo_us_max-servo_us_min);
const int servo_10deg_us=map(10,0,servo_deg_max-servo_deg_min,0,servo_us_max-servo_us_min);
const int servo_20deg_us=map(20,0,servo_deg_max-servo_deg_min,0,servo_us_max-servo_us_min);
const int servo_30deg_us=map(30,0,servo_deg_max-servo_deg_min,0,servo_us_max-servo_us_min);
const int servo_40deg_us=map(40,0,servo_deg_max-servo_deg_min,0,servo_us_max-servo_us_min);
const int servo_45deg_us=map(45,0,servo_deg_max-servo_deg_min,0,servo_us_max-servo_us_min);
const int servo_50deg_us=map(50,0,servo_deg_max-servo_deg_min,0,servo_us_max-servo_us_min);
const int servo_60deg_us=map(60,0,servo_deg_max-servo_deg_min,0,servo_us_max-servo_us_min);

volatile int rudder = 0;
volatile int elevator = 0;
volatile int aileron = 0;
volatile int throttle = 0;
volatile int frequency = 0;
volatile int servo_adjust_left = 0;
volatile int servo_adjust_right = 0;
int freq_tmp = 0;

Servo servo_left;
Servo servo_right;
int servo_left_val = 0;
int servo_right_val = 0;

volatile float y = 0.0;
float y_old = 0.0;
float y_cos = 0.0;
float y_cos_old = 0.0;


unsigned long t = 0;
unsigned long time_now = 0;
unsigned long t_record = 0;
float phi = 0.0;

void on_timer_sin(){
  time_now = millis();

  if(freq_tmp!=frequency){  // 周波数の指令値が変わった際に連続的にsin波を切り替える
    phi = atan2(y_old,y_cos_old);
    freq_tmp = frequency;
    t_record = time_now-dt_sin;
  }

  t = time_now-t_record;
  y=sin(phi+2.0*PI*((float)freq_tmp/100*t/1000));
  y_cos=cos(phi+2.0*PI*((float)freq_tmp/100*t/1000));

  y_old = y;
  y_cos_old = y_cos;

  if(flag_square_wave == 1){ //矩形波を作る
    if(y>=0.0) y=1.0;
    else y=-1.0;
  }  
}
void on_timer_servo(){
  servo_left_val =  (int)(servo_us_neutral+(-elevator+aileron+servo_adjust_left)+(throttle+rudder)*y);
  servo_right_val = (int)(servo_us_neutral+(elevator+aileron+servo_adjust_right)+(-throttle+rudder)*y);

  servo_left.writeMicroseconds(servo_left_val);
  servo_right.writeMicroseconds(servo_right_val);
}

void setup() {
  Serial.begin(115200);
  sbus_rx.Begin();

  servo_left.setPeriodHertz(servo_period);
  servo_right.setPeriodHertz(servo_period);
 
  servo_left.attach(servo_left_pin,servo_us_min,servo_us_max);
  servo_right.attach(servo_right_pin,servo_us_min,servo_us_max);
  
  delay(2000);
  timer_sin.attach_ms(dt_sin,on_timer_sin);
  timer_servo.attach_ms(dt_servo,on_timer_servo);
}

//352~1696 配列番号ch0(変数ch1) 右レバー　左右　J1   標準プロポチャンネルch1
//1707~363 配列番号ch1(変数ch2) 左レバー　上下　J3   標準プロポチャンネルch2
//352~1696 配列番号ch2(変数ch3) 右レバー　上下　J2   標準プロポチャンネルch3
//360~1704 配列番号ch3(変数ch4) 左レバー　左右　J4   標準プロポチャンネルch4
//352,1024,1696 配列番号ch4                         標準プロポチャンネルch5
//1696~352 配列番号ch5(変数ch5) T10Jの場合はVR  T12Kの場合はLS
//352~1696 配列番号ch6(変数ch6) T12Kの場合はLD
//352~1696 配列番号ch7(変数ch7) T12Kの場合はRD
//T10J, T12Kの場合は
//propo_neutral: 1024
//propo_min: 352
//propo_max: 1696

void loop() {
  if (sbus_rx.Read()) {
    data = sbus_rx.data();
    aileron =  map(constrain(data.ch[0],propo_min,propo_max),propo_min,propo_max,-servo_30deg_us,servo_30deg_us); //-30～30度のマイクロ秒
    elevator = map(constrain(data.ch[1],propo_min,propo_max),propo_min,propo_max,-servo_40deg_us,servo_40deg_us); //-40～40度のマイクロ秒
    throttle = map(constrain(data.ch[2],propo_min+100,propo_max),propo_min+100,propo_max,servo_0deg_us,servo_60deg_us);   //羽ばたき角度　0～60度 不感帯あり
    rudder =   map(constrain(data.ch[3],propo_min,propo_max),propo_min,propo_max,-servo_30deg_us,servo_30deg_us); //-30～30度のマイクロ秒
    frequency =map(constrain(data.ch[5],propo_min,propo_max),propo_min,propo_max,1000,0); //分解能を上げるために10Hzを1000にする
    servo_adjust_left = map(constrain(data.ch[6],propo_min,propo_max),propo_min,propo_max,-servo_10deg_us,servo_10deg_us); //左サーボの水平調整-10～10度のマイクロ秒
    servo_adjust_right = map(constrain(data.ch[7],propo_min,propo_max),propo_min,propo_max,-servo_10deg_us,servo_10deg_us);//右サーボの水平調整-10～10度のマイクロ秒

    if(data.ch[4]>propo_neutral) flag_square_wave = 1; 
    else flag_square_wave = 0;  //sin波

    //  //データ表示用
    // for (int8_t i = 0; i < 8; i++) {
    //   Serial.print(data.ch[i]);
    //   Serial.print("\t");
    // }
    // Serial.println();

//     Serial.print("ail:");Serial.print(aileron);
//     Serial.print(",\t");
//     Serial.print("ele:");Serial.print(elevator);
//     Serial.print(",\t");
//     Serial.print("thr:");Serial.print(throttle);
//     Serial.print(",\t");
//     Serial.print("rud:");Serial.print(rudder);
//     Serial.print(",\t");
//     Serial.print("freq:");Serial.print((float)frequency/100);
//     Serial.print(",\t");
//     Serial.print("y:");Serial.print(y);
//     Serial.print(",\t");
//     Serial.print("servol:");Serial.print(servo_left_val);
//     Serial.print(",\t");
      Serial.print("Aileron:");
      Serial.print(aileron);
      Serial.print(",");
      Serial.print("Elevator:");
      Serial.print(elevator);
      Serial.print(",");
      Serial.print("Throttle:");
      Serial.print(throttle);
      Serial.print(",");
      Serial.print("Rudder:");
      Serial.print(rudder);
      Serial.print(",");
      Serial.print("Freq:");
      Serial.print((float)frequency/100);
      Serial.print(",");
      Serial.print("servo_adjust_left:");
      Serial.print(servo_adjust_left);
      Serial.print(",");
      Serial.print("servo_adjust_right");
      Serial.println(servo_adjust_right);

    // Serial.print("servo_center:");Serial.print((float)map(servo_left_val*10,servo_us_min*10,servo_us_max*10,servo_deg_min*10,servo_deg_max*10)/10);
    // Serial.print(",\n");
  }
}
