#include <math.h>
// プログラム作成：九工大大竹研
//参考：2サーボ駆動型羽ばたき機のArduino CODE（2　Servo）http://kakutaclinic.life.coocan.jp/SFOsys2S.html
#include<ESP32Servo.h>  // https://github.com/madhephaestus/ESP32Servo
#include "sbus.h"       // https://github.com/bolderflight/sbus
#include <Ticker.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <wit_c_sdk.h>
#include <Wire.h>
#include <REG.h>
#define PI 3.141592653589793

///// WiFi settings ///////////
//const char* ssid = "ohtake_lab_2G";
//const char* password = "gG5pTe593sF26Hw";
//const IPAddress ip(192, 168, 1, 150);      //IP address of Micro Computer
//const IPAddress gateway(192, 168, 1, 1);   //Set gateway address
//const IPAddress subnet(255, 255, 255, 0);  //Set subnet mask
//const IPAddress dns1(192, 168, 1, 1);      //Set DNS address
//static const char *kRemoteIpadr = "192.168.1.72";  //Your PC's IP address
//static const int kRmoteUdpPort = 10000; //Your PC's receive port number
//static const int kLocalPort = 5000;  //Sending Port number of Micro Computer
//static WiFiUDP wifiUdp; 
//double send_data[30];
//////////////////////////////
const char* ssid = "elecom-b61b9f";
const char* password = "dh6r4npik7np";
const IPAddress ip(192, 168, 1, 150);      //IP address of Micro Computer
const IPAddress gateway(192, 168, 2, 1);   //Set gateway address
const IPAddress subnet(255, 255, 255, 0);  //Set subnet mask
const IPAddress dns1(192, 168, 2, 1);      //Set DNS address
static const char *kRemoteIpadr = "192.168.2.100";  //Your PC's IP address
static const int kRmoteUdpPort = 10000; //Your PC's receive port number
static const int kLocalPort = 5000;  //Sending Port number of Micro Computer
static WiFiUDP wifiUdp; 
double send_data[30];


//const char* ssid = "Aritad_P";
//const char* password = "11140789";
//const IPAddress ip(192, 168, 1, 150);      //IP address of Micro Computer
//const IPAddress gateway(192, 168, 0, 1);   //Set gateway address
//const IPAddress subnet(255, 255, 255, 0);  //Set subnet mask
//const IPAddress dns1(192, 168, 0, 1);      //Set DNS address
//static const char *kRemoteIpadr = "192.168.0.233";  //Your PC's IP address
//static const int kRmoteUdpPort = 10000; //Your PC's receive port number
//static const int kLocalPort = 5000;  //Sending Port number of Micro Computer
//static WiFiUDP wifiUdp; 
//double send_data[30];



bfs::SbusRx sbus_rx(&Serial1,36,18,true);  //Rx:36, Tx:18 Inverted serial
bfs::SbusData data;

const int LED_PIN = 27;  //for pico 27, for s3 21
Ticker timer_sin;
Ticker timer_servo;
Ticker timer_senddata;

const int dt_senddata = 5;

/////// Servo settings //////////////////////////////////////////////
const int flag_square_wave = 0;    //矩形波の場合は1，Sin波の場合は1以外．
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
unsigned long time_now = 0, T=0,s=0;
unsigned long t_record = 0;
float phi = 0.0;

//IMUの変数定義
#define ACC_UPDATE    0x01
#define GYRO_UPDATE   0x02
#define ANGLE_UPDATE  0x04
#define MAG_UPDATE    0x08
#define READ_UPDATE   0x80
static char s_cDataUpdate = 0, s_cCmd = 0xff;

static void CmdProcess(void);
static void AutoScanSensor(void);
static void ScanSensor(int i);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
static int32_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length);
static int32_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t* data, uint32_t length);
static void Delayms(uint16_t ucMs);

int Air_Pressure;
volatile float fAcc[3], fGyro[3], fAngle[3];
int display_timing = 0;

int i;


volatile float  Height, e_height=0, e_height_pre=0, de_height=0, height0=0;

volatile int flag_r = 0, flag_p = 0, flag_y = 0, flag_h=0, c_r=0, c_p=0, c_y=0;

volatile float  KP_GAIN_r = 10, KI_GAIN_r = 0, KD_GAIN_r = 0,
                KP_GAIN_p = 15,  KI_GAIN_p = 0, KD_GAIN_p = 0,
                KP_GAIN_y = 20, KI_GAIN_y = 0, KD_GAIN_y = 0,
                KP_GAIN_h = 65, KI_GAIN_h = 0, KD_GAIN_h = 0;

volatile float  e_roll   = 0, de_roll  = 0, u_roll  = 0,
                e_pitch  = 0, de_pitch = 0, u_pitch = 0,
                e_yaw    = 0, de_yaw   = 0, u_yaw   = 0,
                u_height = 0,u_height_pre = 0;

volatile float sum_r=0, sum_p=0, sum_y=0, roll=0;
//////////////////////

float var;          // Measured value
float filtered_var;
//Kalman Filter
float x_est_last = 0;      // Last estimated state
float P_last = 1;          // Last estimated covariance
float Q = 0.025;           // Process noise covariance //0.022
float R = 0.75;           // Measurement noise covariance //0.617
float K;                   // Kalman gain
float P;                   // Updated covariance
float x_temp_est;          // A priori estimate
float P_temp;              // A priori covariance
float z_measured;          // Measured value
float x_est;               // Estimated value
//Moving AVG Filter
double val_volt = 0.00;
double vout_fit = 0.00;
const int SMOOTHING_WINDOW_SIZE_position = 15; // Number of sampling data.
int _samples_position[SMOOTHING_WINDOW_SIZE_position];
int _curReadIndex_position = 0;
double _sampleTotal_position  = 0.00;
double _sampleAvg_position = 0.00;
double position_max = 0.00;
double busposition;

float filter_angleY;
float filter_gyroY;

float filter_roll;
float filter_deRoll;

float filter_yaw;
float filter_deYaw;
//float Pitch_Setpoint=0;
int Error_Pitch;
float Pitch_controller_Input;
float Pitch_controller_Output;
int Member_Pitch;


volatile float prev_u_pitch,u_pitch_control;

float u_pitch_Error,u_pitch_dError,FZ_dError,pitch_dError;
float Fz_Height_dError;


float u_roll_deError,u_roll_Error;
float u_yaw_Error,u_yaw_deError;
volatile int FZ_elevator = 0;
float TH_Wing=100;

float u_height_Error,u_height_dError;

volatile float amplitude_right=1,amplitude_left=1;
volatile float amplitude_RCM,amplitude_LCM;
volatile float y_left = 0;
volatile float y_right = 0;

volatile float height_setpoint,roll_setpoint,pitch_setpoint,yaw_setpoint;
static bool initial_height = false;

float Height_control_Output=0;
float mov_avg(float busposition){
   _sampleTotal_position = _sampleTotal_position - _samples_position[_curReadIndex_position];
  _samples_position[_curReadIndex_position] = busposition;
  _sampleTotal_position= _sampleTotal_position + _samples_position[_curReadIndex_position];
  _curReadIndex_position = _curReadIndex_position + 1;
  if (_curReadIndex_position >= SMOOTHING_WINDOW_SIZE_position) {
    _curReadIndex_position = 0;
  }
  _sampleAvg_position = _sampleTotal_position / SMOOTHING_WINDOW_SIZE_position;
  return _sampleAvg_position;
  
  }
float kalmanUpdate(float measurement) {
  // Step 1: Prediction update
  x_temp_est = x_est_last;       // x_temp_est = x_est_last
  P_temp = P_last + Q;           // P_temp = P_last + Q
  
  // Step 2: Measurement update
  K = P_temp / (P_temp + R);     // K = P_temp / (P_temp + R)
  x_est = x_temp_est + K * (measurement - x_temp_est);  // x_est = x_temp_est + K * (z - x_temp_est)
  P = (1 - K) * P_temp;          // P = (1 - K) * P_temp
  
  // Update variables for next iteration
  P_last = P;
  x_est_last = x_est;

  return x_est;   // Return the estimated value
}

void on_timer_sin(){
  unsigned long time_now = millis();

  if(freq_tmp != frequency){
    phi = atan2(y_old, y_cos_old);
    freq_tmp = frequency;
    t_record = time_now - dt_sin;
    }

  t = time_now - t_record;
  
  float phase = phi + 2.0 * PI * ((float)freq_tmp / 100 * t / 1000.0);
  
  y_left = amplitude_LCM * sin(phase);
  y_right = amplitude_RCM * sin(phase);
  
  y_cos = cos(phase);
  
  y_old = (y_left + y_right) / 2.0;     
  y_cos_old = y_cos;

}
float fuzzyKD_Pitch(float dErr) {
    float absDE = fabs(dErr);
    float kd;
    const float KD_BASE = 0.4f;
    const float KD_SLOW = 1.5f * KD_BASE;  
    const float KD_MID  = 1.0f * KD_BASE;  
    const float KD_FAST = 0.5f * KD_BASE;  

    if (absDE < 5.0f)        return KD_SLOW; 
    else if (absDE < 50.0f)  return KD_MID;  
    else                     return KD_FAST; 
    return kd;
}

float fuzzyKD_Height(float dErr) { /////max 60
    float absDE = fabs(dErr);

   
    const float KD_BASE = 0.25f;
    const float KD_SLOW = 2.0f * KD_BASE;  // 0.60
    const float KD_MID  = 1.0f * KD_BASE;  // 0.40
    const float KD_FAST = 0.25 * KD_BASE;  // 0.20

    if (absDE < 30.0f)        return KD_SLOW; // ช้า → หน่วงมากขึ้น
    else if (absDE < 60.0f)  return KD_MID;  // กลาง → เท่าเดิม
    else                     return KD_FAST; // เร็ว → ลดหน่วงกันสั่น
}

float fuzzyKD_Yaw(float dErr) { ///max 2000
    float absDE = fabs(dErr);

   
    const float KD_BASE = 0.001f;
    const float KD_SLOW = 0.005f * KD_BASE;  
    const float KD_MID  = 0.002f * KD_BASE;  
    const float KD_FAST = 0.001f * KD_BASE;  

    if (absDE < 70.0f)        return KD_SLOW; // ช้า → หน่วงมากขึ้น
    else if (absDE < 180.0f)  return KD_MID;  // กลาง → เท่าเดิม
    else                     return KD_FAST; // เร็ว → ลดหน่วงกันสั่น
}
float TriangularMF(float x, float left, float peak, float right) {
  return max(min((x - left) / (peak - left), (right - x) / (right - peak)), 0.0f);
}

float TrapezoidalMF(float x, float leftStart, float leftTop, float rightTop, float rightEnd) {
  return max(min(min((x - leftStart) / (leftTop - leftStart), 1.0f), (rightEnd - x) / (rightEnd - rightTop)), 0.0f);
}

float GaussianMF(float x, float center, float width) {
  return exp(-0.5f * pow((x - center) / width, 2));
}

float BellMF(float x, float width, float slope, float center) {
  return 1.0 / (1.0 + pow(fabs((x - center) / width), 2 * slope));
}

float SigmoidMF(float x, float slope, float center) {
  return 1.0 / (1.0 + exp(-slope * (x - center)));
}

float ZShapeMF(float x, float start, float end) {
  if (x <= start) return 1.0;
  else if (x >= end) return 0.0;
  float t = (x - start) / (end - start);
  return 1 - 2 * t * t;
}

float SShapeMF(float x, float start, float end) {
  if (x <= start) return 0.0;
  else if (x >= end) return 1.0;
  float t = (x - start) / (end - start);
  return 2 * t * t;
}

volatile bool runFuzzyFlag = false;

// ===================== Output Membership Functions ======================
float OutputMF_OPFD(float x) { return ZShapeMF(x, -400, -140); }
float OutputMF_OPLD(float x) { return TriangularMF(x, -250, -100, -20); }
float OutputMF_ZDEG(float x) { return TriangularMF(x, -40, 0, 40); }
float OutputMF_OPLU(float x) { return TriangularMF(x, 20, 100, 250); }
float OutputMF_OPFU(float x) { return SShapeMF(x, 140, 400); }

// ===================== Optimized Fuzzy Control ======================
float fuzzyControl(float error, float dError, float u_height_pre) {
  // --- Fuzzify Inputs ---
  float e_NEHL = ZShapeMF(error, -2, -1);
  float e_NEHS = TriangularMF(error, -1.5, -0.2, 0);
  float e_ZHE  = TriangularMF(error, -0.25, 0.0, 0.25);
  float e_PEHS = TriangularMF(error, 0, 0.2, 1.5);
  float e_PEHL = SShapeMF(error, 1, 2);
  

//  float de_Ps = ZShapeMF(dError, -60, -30);
//  float de_Ze = TriangularMF(dError, -40, 0, 40);
//  float de_Ng = SShapeMF(dError, 30, 60);
  float de_Ps = ZShapeMF(dError, -40, -20);
  float de_Ze = TriangularMF(dError, -25, 0, 25);
  float de_Ng = SShapeMF(dError, 20, 40);
  // --- Define Rule Structure ---
  struct Rule {
    float w;
    float (*mf)(float);
    float constOutput;
    bool useConst;
  };

  Rule rules[20];
  int r = 0;

  // --- Rule Base ---
  rules[r++] = {min(e_NEHL, de_Ze), OutputMF_OPFD, 0, false};
  rules[r++] = {min(e_NEHL, de_Ng), OutputMF_OPFD, 0, false};
  rules[r++] = {min(e_NEHS, de_Ng), OutputMF_OPFD, 0, false};

  rules[r++] = {min(e_ZHE, de_Ng), OutputMF_OPLD, 0, false};
  rules[r++] = {min(e_NEHS, de_Ze), OutputMF_OPLD, 0, false};
  rules[r++] = {min(e_NEHL, de_Ps), OutputMF_OPLD, 0, false};

  rules[r++] = {min(e_PEHS, de_Ng), NULL, u_height_pre, true};
  rules[r++] = {min(e_ZHE,  de_Ze), OutputMF_ZDEG, 0, false};
  rules[r++] = {min(e_NEHS, de_Ps), NULL, u_height_pre, true};

  rules[r++] = {min(e_PEHL, de_Ng), OutputMF_OPLU, 0, false};
  rules[r++] = {min(e_PEHS, de_Ze), OutputMF_OPLU, 0, false};
  rules[r++] = {min(e_ZHE,  de_Ps), OutputMF_OPLU, 0, false};

  rules[r++] = {min(e_PEHL, de_Ps), OutputMF_OPFU, 0, false};
  rules[r++] = {min(e_PEHL, de_Ze), OutputMF_OPFU, 0, false};
  rules[r++] = {min(e_PEHS, de_Ps), OutputMF_OPFU, 0, false};

  // --- Aggregation + Defuzzification ---
  float num = 0, den = 0;

  // 1. Constant-output rules
  for(int i=0;i<r;i++){
    if(rules[i].useConst){
      num += rules[i].constOutput * rules[i].w;
      den += rules[i].w;
    }
  }

  // 2. Membership-based rules (coarse step = faster)
  for(float x=-400; x<=400; x+=40.0){  // step=40 ทำให้เร็วขึ้น
    float mu = 0;
    for(int i=0;i<r;i++){
      if(!rules[i].useConst){
        mu = fmax(mu, fmin(rules[i].w, rules[i].mf(x)));
      }
    }
    if(mu>0){
      num += x*mu;
      den += mu;
    }
    yield(); // ป้องกัน watchdog
  }

  return (den==0)?0:num/den;
}



void on_timer_servo(){

//      if(data.ch[10] >= 1500) {      
//        e_height = height_setpoint-Height;
//        de_height = (e_height - e_height_pre)/0.005;
//        //u_height_Error=((-0.461299)*(pow(e_height,3)))+(e_height*53.231828); /////Kp
//        u_height_Error=(((-0.461299)*(pow(e_height,3)))+(e_height*175)); ///neww/////Kp
//        Fz_Height_dError=fuzzyKD_Height(de_height);
//        u_height_dError=Fz_Height_dError*de_height;  /////Kd
//        u_height=u_height_Error+u_height_dError;
//        //u_height=u_height_Error;
//        if(0>e_height){
//          u_height=abs(u_height);
//          }
//        else  
//          u_height=u_height*(-1);
//        if (u_height > 400) {
//            u_height = 400;
//          } 
//        else if (u_height < -400) {
//            u_height = -400;
//        }
//        e_height_pre = e_height;
//      }
//      else{
//        u_height=0;
//        height_setpoint= Height;
//      }
      runFuzzyFlag = true;


      if(data.ch[4]>=1500){
        ////e_pitch=+pitch_setpoint+filter_angleY;//////old
        ////e_pitch=-pitch_setpoint+filter_angleY;//////new
        e_pitch=pitch_setpoint-filter_angleY;//////new
        Pitch_controller_Input=e_pitch;
        pitch_dError=kalmanUpdate(fGyro[0]);
        FZ_dError=fuzzyKD_Pitch(pitch_dError);
        
        u_pitch_dError= FZ_dError*kalmanUpdate(fGyro[0]);
        //u_pitch_Error = ((-0.0004)*(pow(Pitch_controller_Input,3)))+(4.5694*Pitch_controller_Input); ////From fuzzy Error like Kp
        //u_pitch_Error = ((-0.0004)*(pow(Pitch_controller_Input,3)))+(15*Pitch_controller_Input); ////From fuzzy Error like Kp
        u_pitch_Error = ((-0.0004)*(pow(Pitch_controller_Input,3)))+(5*Pitch_controller_Input); ////From fuzzy Error like Kp
        u_pitch=u_pitch_Error+u_pitch_dError;
        ///// pitch setup
          if(u_pitch>400)
          {
            u_pitch = 400;
            }
          else if(-400>u_pitch)
          {
            u_pitch = -400;
            }
          }
       else{
        pitch_setpoint=filter_angleY;
        u_pitch=0;
        }
       if(data.ch[8]>=1500){
        ///e_roll= fAngle[1]-roll_setpoint;////old
        e_roll= roll_setpoint-fAngle[1];////
        ////u_roll_Error=((-0.001550)*(pow(e_roll,3)))+(7.522670*e_roll);  ///Fz like Kp ////////////tune this///////////////
        u_roll_Error=((-0.001550)*(pow(e_roll,3)))+(7.522670*e_roll);  ///Fz like Kp
        ////ex y = -0.001700*x^3 + 0.000000*x^2 + 7.744940*x + -0.000000
        //// y = -0.001897*x^3 + 0.000000*x^2 + 8.031370*x + -0.000000
        //// y = -0.001754*x^3 + 0.000000*x^2 + 7.820982*x + -0.000000
        //// y = -0.000914*x^3 + 0.000000*x^2 + 6.551010*x + -0.000000
        /// more agressive y = -0.001700*x^3 + 0.000000*x^2 + 7.744940*x + -0.00000
        /// = -0.001944*x^3 + 0.000000*x^2 + 8.101337*x + -0.000000
        u_roll_deError=fGyro[2]*0.000; ///Fz like Kd/////no kd for Roll////////////////
        u_roll=u_roll_Error+u_roll_deError;
        u_roll = abs(u_roll_Error);   
        if (e_roll < 0) {
        u_roll = -u_roll;         
            }     
        }
       else{
         u_roll=0;
         roll_setpoint=fAngle[1];
        }
      if(data.ch[9]>=1500){
        e_yaw = yaw_setpoint-fAngle[2];
  //      u_yaw_Error=((-0.000006)*(pow(e_yaw,3)))+(0.084141*e_yaw);////only kp
        u_yaw_Error=(0.084141*e_yaw);////only kp //////////tune this/////////////////////////
        u_yaw_deError=fuzzyKD_Yaw(fGyro[1])*fGyro[1];
        u_yaw=u_yaw_Error+u_yaw_deError;
        if(u_yaw>3.8){
          u_yaw=3.8;
          }
        else if(-3.8>u_yaw){
          u_yaw=-3.8;
          }
          
        if(0>e_yaw){
          amplitude_RCM=abs(u_yaw)+amplitude_right;
          amplitude_LCM=1;
          }
        else if(e_yaw>0){
          amplitude_LCM=abs(u_yaw)+amplitude_left;
          amplitude_RCM=1;
          }
      }
      else{
        
        yaw_setpoint=fAngle[2];
        u_yaw=1;////////
        amplitude_RCM=1;/////
        amplitude_LCM=1;/////
        }  
  servo_left_val = constrain(servo_left_val, servo_us_min, servo_us_max);
  servo_right_val = constrain(servo_right_val, servo_us_min, servo_us_max);
  
  servo_left_val  = (int)(servo_us_neutral+(-FZ_elevator-elevator-u_pitch+aileron+u_roll-u_height+servo_adjust_left )+( throttle+rudder)*y_left);//// at this side is right 
  servo_right_val = (int)(servo_us_neutral+(FZ_elevator+elevator+u_pitch+aileron+u_roll+u_height+servo_adjust_right)+(-throttle+rudder)*y_right); //// at this side is left

  servo_left.writeMicroseconds(servo_left_val);
  servo_right.writeMicroseconds(servo_right_val);
  
}


void on_timer_senddata(){
  unsigned long time_now = millis();
  wifiUdp.beginPacket(kRemoteIpadr, kRmoteUdpPort);
  wifiUdp.print(time_now);//1
  wifiUdp.print(" ");
  wifiUdp.print(Air_Pressure);//2
  wifiUdp.print(" ");
  wifiUdp.print(Height);//3
  wifiUdp.print(" ");
  wifiUdp.print(e_height);//4
  wifiUdp.print(" ");
  wifiUdp.print(rudder);//5
  wifiUdp.print(" ");
  wifiUdp.print(elevator);//6
  wifiUdp.print(" ");
  wifiUdp.print(aileron);//7
  wifiUdp.print(" ");
  wifiUdp.print(throttle);//8
  wifiUdp.print(" ");
  wifiUdp.print(frequency);//9
  wifiUdp.print(" ");
  wifiUdp.print(servo_left_val);//10
  wifiUdp.print(" ");
  wifiUdp.print(servo_right_val);//11
  wifiUdp.print(" ");
  wifiUdp.print(data.ch[4]);//12
  wifiUdp.print(" ");
  wifiUdp.print(data.ch[8]);
  wifiUdp.print(" ");
  wifiUdp.print(data.ch[9]);
  wifiUdp.print(" ");
  wifiUdp.print(data.ch[10]);
  wifiUdp.print(" ");
  wifiUdp.print(fGyro[1],4);
  wifiUdp.print(" ");
  wifiUdp.print(fGyro[2],4);
  wifiUdp.print(" ");
  wifiUdp.print(fAngle[0],4);//18pitch
  wifiUdp.print(" ");
  wifiUdp.print(fAngle[1],4);//19roll
  wifiUdp.print(" ");
  wifiUdp.print(fAngle[2],4);//20 yaw
  wifiUdp.print(" ");
  wifiUdp.endPacket(); 
//    wifiUdp.beginPacket(kRemoteIpadr, kRmoteUdpPort);
//    wifiUdp.print(e_pitch);
//    wifiUdp.print(" ");
//    wifiUdp.print(filter_angleY);
//    wifiUdp.print(" ");
//    wifiUdp.print(filter_gyroY);
//    wifiUdp.print(" ");
//    wifiUdp.print(servo_right_val);
//    wifiUdp.print(" ");
//    wifiUdp.print(servo_left_val);
//    wifiUdp.print(" ");
//    wifiUdp.print(aileron);
//    wifiUdp.print(" ");
//    wifiUdp.print(elevator);
//    wifiUdp.print(" ");
//    wifiUdp.print(throttle);
//    wifiUdp.print(" ");
//    wifiUdp.print(rudder);
//    wifiUdp.print(" ");
//    wifiUdp.print(frequency);
//    wifiUdp.print(" ");
//    wifiUdp.print(fAcc[0],4);
//    wifiUdp.print(" ");
//    wifiUdp.print(fAcc[1],4);
//    wifiUdp.print(" ");
//    wifiUdp.print(fAcc[2],4);
//    wifiUdp.print(" ");
//    wifiUdp.print(Height);
//    wifiUdp.print(" ");
//    wifiUdp.print(Height-height0);
//    wifiUdp.print(" ");
//    wifiUdp.print(Air_Pressure);
//    wifiUdp.print(" ");
//    wifiUdp.endPacket();
  if(display_timing%20==0)
  {
    Serial.print(time_now);
    Serial.print(", ");
    Serial.print(Air_Pressure);
    Serial.print(", ");
    Serial.print(Height);
    Serial.print(", ");
    Serial.print(e_height);
    Serial.print(", ");
    Serial.print(u_height);
    Serial.print(", ");
    Serial.print(de_height);
    Serial.print(", ");
    Serial.print(aileron);
    Serial.print(", ");
    Serial.print(throttle);
    Serial.print(", ");
    Serial.print(frequency);
    Serial.print(", ");
    Serial.print(servo_left_val);
    Serial.print(", ");
    Serial.print(servo_right_val);
    Serial.print(", ");
    Serial.print(fAcc[0],4);
    Serial.print(", ");
    Serial.print(fAcc[1],4);
    Serial.print(", ");
    Serial.print(fAcc[2],4);
    Serial.print(", ");
    Serial.print(fGyro[0],4);
    Serial.print(", ");
    Serial.print(fGyro[1],4);
    Serial.print(", ");
    Serial.print(fGyro[2],4);
    Serial.print(", ");
    Serial.print(fAngle[0],4);
    Serial.print(", ");
    Serial.print(fAngle[1],4);
    Serial.print(", ");
    Serial.print(fAngle[2],4);
    Serial.println("");
    display_timing = 0;
  }
//    if(display_timing%20==0)
//  {
//      Serial.print(e_pitch);
//      Serial.print(",");
//      Serial.print(filter_angleY);
//      Serial.print(",");
//      Serial.print(u_pitch_Error);
//      Serial.print(",");
//      Serial.print(u_pitch);
//      Serial.print(",");
//      Serial.print(servo_right_val);
//      Serial.print(",");
//      Serial.print(servo_left_val);
//      Serial.print(",");
//      Serial.print(e_height);
//      Serial.print(",");
//      Serial.println(u_height);
//    display_timing = 0;
//  }
  display_timing = display_timing + 1;
}

bool fuzzyReady = false; // Flag ป้องกัน fuzzyControl รันก่อนพร้อม

void setup() {
  neopixelWrite(LED_PIN, 0, 100, 0);

  Serial.begin(115200);
  sbus_rx.Begin();

  servo_left.setPeriodHertz(servo_period);
  servo_right.setPeriodHertz(servo_period);
 
  servo_left.attach(servo_left_pin, servo_us_min, servo_us_max);
  servo_right.attach(servo_right_pin, servo_us_min, servo_us_max);

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // --- WiFi setup (non-blocking-friendly) ---
  WiFi.mode(WIFI_MODE_STA);
  if (!WiFi.config(ip, gateway, subnet, dns1)) {
      Serial.println("Failed to configure WiFi!");
  }
  WiFi.begin(ssid, password);

  unsigned long startMillis = millis();
  const unsigned long timeout = 20000; // 20s timeout
  while (WiFi.status() != WL_CONNECTED && (millis() - startMillis) < timeout) {
      Serial.print(".");
      vTaskDelay(500 / portTICK_PERIOD_MS); // แทน delay(500)
      yield(); // ให้ watchdog reset
  }

  if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi connected.");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
  } else {
      Serial.println("\nWiFi connection failed, continuing anyway.");
  }

  wifiUdp.begin(kLocalPort);

  // --- I2C setup ---
  Wire.begin();
  Wire.setClock(400000);

  WitInit(WIT_PROTOCOL_I2C, 0x50);
  WitI2cFuncRegister(IICwriteBytes, IICreadBytes);
  WitRegisterCallBack(CopeSensorData);
  WitDelayMsRegister(Delayms);
  Serial.print("\r\n********************** wit-motion IIC ************************\r\n");
  ScanSensor(0x28);
  if (WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK) Serial.print("\r\nSet Bandwidth Error\r\n");

  neopixelWrite(LED_PIN, 0, 0, 0);

  delay(500); // ให้บอร์ด stabilise
  fuzzyReady = true; // ตอนนี้ fuzzyControl สามารถรันได้

  // --- Timer setup ---
  timer_sin.attach_ms(dt_sin, on_timer_sin);
  timer_servo.attach_ms(dt_servo, on_timer_servo);
  timer_senddata.attach_ms(dt_senddata, on_timer_senddata);
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
  int i;
  //WitReadReg(AX, 12);
  WitReadReg(AX, 21);
  if(s_cDataUpdate)
  {
    for(i = 0; i < 3; i++)
    {
      fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
      fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
      fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
    }
    Air_Pressure = ((int)sReg[PressureH]<<16|sReg[PressureL]);
    Height = ((int)sReg[HeightH]<<16|sReg[HeightL])*0.01;
    fAngle[0] = fAngle[0] - 90;
    if(s_cDataUpdate & ACC_UPDATE)
    {
      filter_angleY=mov_avg(fAngle[0]);//////////////update data real time
      filter_gyroY=kalmanUpdate(fGyro[0]);///pitch
      
/*      Serial.print("acc:");
      Serial.print(fAcc[0], 3);
      Serial.print(" ");
      Serial.print(fAcc[1], 3);
      Serial.print(" ");
      Serial.print(fAcc[2], 3);
//      Serial.print("\r\n");
*/      s_cDataUpdate &= ~ACC_UPDATE;
    }
    if(s_cDataUpdate & GYRO_UPDATE)
    {
/*      Serial.print("gyro:");
      Serial.print(fGyro[0], 1);
      Serial.print(" ");
      Serial.print(fGyro[1], 1);
      Serial.print(" ");
      Serial.print(fGyro[2], 1);
//      Serial.print("\r\n");
*/      s_cDataUpdate &= ~GYRO_UPDATE;
    }
    if(s_cDataUpdate & ANGLE_UPDATE)
    {
/*      Serial.print("angle:");
      Serial.print(fAngle[0], 3);
      Serial.print(" ");
      Serial.print(fAngle[1], 3);
      Serial.print(" ");
      Serial.print(fAngle[2], 3);
//      Serial.print("\r\n");
*/      s_cDataUpdate &= ~ANGLE_UPDATE;
    }
    if(s_cDataUpdate & MAG_UPDATE)
    {
/*      Serial.print("mag:");
      Serial.print(sReg[HX]);
      Serial.print(" ");
      Serial.print(sReg[HY]);
      Serial.print(" ");
      Serial.print(sReg[HZ]);
      Serial.print("\r\n");
*/      s_cDataUpdate &= ~MAG_UPDATE;
    }
    s_cDataUpdate = 0;
    if(data.ch[10] >= 1500) {
    if(runFuzzyFlag){
    runFuzzyFlag = false;

    e_height = height_setpoint - Height;
    de_height = (e_height - e_height_pre)/0.005;

    float control = fuzzyControl(e_height, de_height, u_height_pre);
    Height_control_Output = constrain(control, -400, 400);
    u_height = Height_control_Output;
    u_height_pre = u_height;
    e_height_pre = e_height;
    }
   } 
   else{
        e_height=0;
        u_height=0;
        height_setpoint= Height;
      }
  }

  if (sbus_rx.Read()) {
    data = sbus_rx.data();
    aileron =  map(constrain(data.ch[0],propo_min,propo_max),propo_min,propo_max,-servo_30deg_us,servo_30deg_us); //-30～30度のマイクロ秒
    elevator = map(constrain(data.ch[1],propo_min,propo_max),propo_min,propo_max,-servo_40deg_us,servo_40deg_us); //-40～40度のマイクロ秒
    throttle = map(constrain(data.ch[2],propo_min+100,propo_max),propo_min+100,propo_max,servo_0deg_us,servo_60deg_us);   //羽ばたき角度　0～60度 不感帯あり
    rudder =   map(constrain(data.ch[3],propo_min,propo_max),propo_min,propo_max,-servo_30deg_us,servo_30deg_us); //-30～30度のマイクロ秒
    frequency =map(constrain(data.ch[5],propo_min,propo_max),propo_min,propo_max,1000,0); //分解能を上げるために10Hzを1000にする
    servo_adjust_left = map(constrain(data.ch[6],propo_min,propo_max),propo_min,propo_max,-servo_10deg_us,servo_10deg_us); //左サーボの水平調整-10～10度のマイクロ秒
    servo_adjust_right = map(constrain(data.ch[7],propo_min,propo_max),propo_min,propo_max,-servo_10deg_us,servo_10deg_us);//右サーボの水平調整-10～10度のマイクロ秒

    //if(data.ch[4]>propo_neutral) flag_square_wave = 1; 
    //else flag_square_wave = 0;  //sin波

    //  //データ表示用
    // for (int8_t i = 0; i < 8; i++) {
    //   Serial.print(data.ch[i]);
    //   Serial.print("\t");
    // }
    // Serial.println();

    // Serial.print("ail:");Serial.print(aileron);
    // Serial.print(",\t");
    // Serial.print("ele:");Serial.print(elevator);
    // Serial.print(",\t");
    // Serial.print("thr:");Serial.print(throttle);
    // Serial.print(",\t");
    // Serial.print("rud:");Serial.print(rudder);
    // Serial.print(",\t");
    // Serial.print("freq:");Serial.print((float)frequency/100);
    // Serial.print(",\t");
    // Serial.print("y:");Serial.print(y);
    // Serial.print(",\t");
    // Serial.print("servol:");Serial.print(servo_left_val);
    // Serial.print(",\t");
    // Serial.print("servo_center:");Serial.print((float)map(servo_left_val*10,servo_us_min*10,servo_us_max*10,servo_deg_min*10,servo_deg_max*10)/10);
    // Serial.print(",\n");
  }
}

void CopeCmdData(unsigned char ucData)
{
  static unsigned char s_ucData[50], s_ucRxCnt = 0;
  
  s_ucData[s_ucRxCnt++] = ucData;
  if(s_ucRxCnt<3)return;                    //Less than three data returned
  if(s_ucRxCnt >= 50) s_ucRxCnt = 0;
  if(s_ucRxCnt >= 3)
  {
    if((s_ucData[1] == '\r') && (s_ucData[2] == '\n'))
    {
      s_cCmd = s_ucData[0];
      memset(s_ucData,0,50);
      s_ucRxCnt = 0;
    }
    else 
    {
      s_ucData[0] = s_ucData[1];
      s_ucData[1] = s_ucData[2];
      s_ucRxCnt = 2;
      
    }
  }
}

static int32_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length)
{
  int val;
    Wire.beginTransmission(dev);
    Wire.write(reg);
    Wire.endTransmission(false); //endTransmission but keep the connection active

    val = Wire.requestFrom(dev, length); //Ask for bytes, once done, bus is released by default

  if(val == 0)return 0;
    while(Wire.available() < length) //Hang out until we get the # of bytes we expect
    {
    }

    for(int x = 0 ; x < length ; x++)    data[x] = Wire.read();   

    return 1;
}


static int32_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t* data, uint32_t length)
{
    Wire.beginTransmission(dev);
    Wire.write(reg);
    Wire.write(data, length);
    Wire.endTransmission(); //Stop transmitting

    return 1; 
}

static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum)
{
  int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
            case AZ:
        s_cDataUpdate |= ACC_UPDATE;
            break;
            case GZ:
        s_cDataUpdate |= GYRO_UPDATE;
            break;
            case HZ:
        s_cDataUpdate |= MAG_UPDATE;
            break;
            case Yaw:
        s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
        s_cDataUpdate |= READ_UPDATE;
      break;
        }
    uiReg++;
    }
}

static void Delayms(uint16_t ucMs)
{
  delay(ucMs);
}


static void AutoScanSensor(void)
{
  int i, iRetry;
  
  for(i = 0; i < 0x7F; i++)
  {
    WitInit(WIT_PROTOCOL_I2C, i);
    iRetry = 2;
    do
    {
      s_cDataUpdate = 0;
      WitReadReg(AX, 3);
      delay(5);
      if(s_cDataUpdate != 0)
      {
        Serial.print("find 0x");
        Serial.print(i, HEX);
        Serial.print(" addr sensor\r\n");
        //ShowHelp();
        return ;
      }
      iRetry--;
    }while(iRetry);   
  }
  Serial.print("can not find sensor\r\n");
  Serial.print("please check your connection\r\n");
}

static void ScanSensor(int i)
{
  int iRetry;
  WitInit(WIT_PROTOCOL_I2C, i);
  iRetry = 2;
  do
  {
    s_cDataUpdate = 0;
    WitReadReg(AX, 3);
    delay(5);
    if(s_cDataUpdate != 0)
    {
      Serial.print("find 0x");
      Serial.print(i, HEX);
      Serial.print(" addr sensor\r\n");
      //ShowHelp();
      return ;
    }
    iRetry--;
  }while(iRetry);   
  Serial.print("can not find sensor\r\n");
  Serial.print("please check your connection\r\n");
}
