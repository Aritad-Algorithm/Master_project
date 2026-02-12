#include "FuzzyController.h"
#include "AritadAlgorithm_Yaw.h"

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

volatile int flag_r = 0, flag_p = 0, flag_y = 0, flag_h=0, c_r=0, c_p=0, c_y=0;
volatile float u_roll=0,e_height=0,de_height,Height;


volatile float e_yaw=0,de_yaw=0,u_yaw= 0;

volatile float sum_r=0, sum_p=0, sum_y=0, roll=0;
//////////////////////

float pitch_Setpoint=0;



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
float filter_pitch,filter_roll,filter_yaw;
float pitch_dError,roll_dError,yaw_dError;

FuzzyController heightFuzzy(FuzzyController::HEIGHT_CTRL);
FuzzyController pitchFuzzy (FuzzyController::PITCH_CTRL);
FuzzyController rollFuzzy (FuzzyController::ROLL_CTRL);
AritadAlgorithm_Yaw yawFuzzy(AritadAlgorithm_Yaw::YAW_CTRL);

float u_height=0,u_height_pre=0,e_height_pre=0;
float e_pitch,de_pitch,u_pitch =0,u_pitch_pre =0,e_pitch_pre =0;
float e_pitch_f;
float u_yaw_pre;
float e_roll,e_roll_f;
 
////Kakman filter
class KalmanFilter {
private:
    float x_est_last;   // Last estimated state
    float P_last;       // Last estimated covariance
    float Q;            // Process noise covariance
    float R;            // Measurement noise covariance

public:
    // Constructor
    KalmanFilter(float q, float r, float initial_x = 0.0, float initial_p = 1.0) {
        Q = q;
        R = r;
        x_est_last = initial_x;
        P_last = initial_p;
    }

    // Update function (เหมือน function เดิม)
    float update(float measurement) {
        // Prediction
        float x_temp_est = x_est_last;
        float P_temp = P_last + Q;

        // Measurement update
        float K = P_temp / (P_temp + R);
        float x_est = x_temp_est + K * (measurement - x_temp_est);
        float P = (1 - K) * P_temp;

        // Save state
        x_est_last = x_est;
        P_last = P;

        return x_est;
    }

    // (optional) reset filter
    void reset(float x0 = 0.0, float p0 = 1.0) {
        x_est_last = x0;
        P_last = p0;
    }
};
class MovingAverage {
private:
    int windowSize;
    int *samples;
    int curIndex;
    double total;

public:
    // Constructor
    MovingAverage(int size) {
        windowSize = size;
        samples = new int[windowSize];
        curIndex = 0;
        total = 0.0;

        // init buffer
        for (int i = 0; i < windowSize; i++) {
            samples[i] = 0;
        }
    }

    // Destructor (ป้องกัน memory leak)
    ~MovingAverage() {
        delete[] samples;
    }

    double update(double input) {
        total -= samples[curIndex];
        samples[curIndex] = input;
        total += samples[curIndex];

        curIndex++;
        if (curIndex >= windowSize) {
            curIndex = 0;
        }

        return total / windowSize;
    }

    void reset() {
        total = 0.0;
        curIndex = 0;
        for (int i = 0; i < windowSize; i++) {
            samples[i] = 0;
        }
    }
};


class SlidingModeController {
  public:
    SlidingModeController(float lambda, float K, float boundary) {
      _lambda = lambda;
      _K = K;
      _boundary = boundary;
    }

    float compute(float error, float dError) {
      float s = _lambda * error + dError;
      float u = -_K * sat(s);
      return u;
    }

  private:
    float _lambda;
    float _K;
    float _boundary;

    float sat(float s) {
      if (s > _boundary) return 1.0;
      if (s < -_boundary) return -1.0;
      return s / _boundary;   // smooth region
    }
};

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

KalmanFilter kf_pitch(0.025,0.75);
MovingAverage mov_avg_pitch(15);

KalmanFilter kf_roll(0.025,0.75);
MovingAverage mov_avg_roll(15);

KalmanFilter kf_yaw(0.025,0.75);
MovingAverage mov_avg_yaw(15);

SlidingModeController smc(1,    // lambda
1.0,   // K
1     // boundary
);
void on_timer_servo(){

  servo_left_val = constrain(servo_left_val, servo_us_min, servo_us_max);
  servo_right_val = constrain(servo_right_val, servo_us_min, servo_us_max);
  
  servo_left_val  = (int)(servo_us_neutral+(-elevator-u_pitch+aileron+u_roll-u_height+servo_adjust_left )+( throttle+rudder)*y_left);//// at this side is right 
  servo_right_val = (int)(servo_us_neutral+(+elevator+u_pitch+aileron+u_roll+u_height+servo_adjust_right)+(-throttle+rudder)*y_right); //// at this side is left

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
  wifiUdp.print(e_pitch);//5
  wifiUdp.print(" ");
  wifiUdp.print(e_roll);//6
  wifiUdp.print(" ");
  wifiUdp.print(e_yaw);//7
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
  {///u_pitch
//    Serial.print(time_now);//yaw_dError
//    Serial.print(", ");
    Serial.print(yaw_dError);//yaw_dError
    Serial.print(", ");
    Serial.print(u_yaw);
    Serial.print(", ");
    Serial.print(e_pitch);
    Serial.print(", ");
    Serial.print(u_roll);
    Serial.print(", ");
    Serial.print(e_roll);
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
///float filter_pitch,filter_roll,filter_yaw;
//float pitch_dError,roll_dError,yaw_dError;

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
//    if(s_cDataUpdate & ACC_UPDATE)
//    {
      filter_pitch=mov_avg_pitch.update(fAngle[0]);//////////////update data real time
      pitch_dError=kf_pitch.update(fGyro[0]);
      filter_roll=mov_avg_roll.update(fAngle[1]);
      roll_dError=kf_roll.update(fGyro[2]);
      filter_yaw=mov_avg_yaw.update(fAngle[2]);
      yaw_dError=kf_yaw.update(fGyro[1]);
//*/      s_cDataUpdate &= ~ACC_UPDATE;
//    }
//    s_cDataUpdate = 0;

      if(data.ch[10]>=1500){
    
        float e_height=height_setpoint-Height;
        float de_height=(e_height-e_height_pre)/0.005;
        u_height=constrain(heightFuzzy.compute(e_height,de_height,u_height_pre),-400,400);
        u_height_pre=u_height;
        e_height_pre=e_height; 
      }
      else{
         u_height_pre=0;
         e_height=0;
         u_height=0;
         height_setpoint= Height;
      }
      if(data.ch[4]>=1500){
        ////e_pitch=+pitch_setpoint+filter_angleY;//////old
        ////e_pitch=-pitch_setpoint+filter_angleY;//////new
        e_pitch=pitch_setpoint-filter_pitch;//////new
        e_pitch_f=abs(e_pitch);
        u_pitch=constrain(pitchFuzzy.compute(e_pitch_f,pitch_dError,u_pitch_pre),-400,400);
        u_pitch_pre=u_pitch;
        if (e_pitch < 0) {
        u_pitch = -u_pitch;         
            }   



        
          }
       else{
          e_pitch=0;
          u_pitch=0;
          pitch_setpoint=filter_pitch;
          u_pitch =u_pitch_pre=0;
        }
       if(data.ch[8]>=1500){
        ///e_roll= fAngle[1]-roll_setpoint;////old
        e_roll=roll_setpoint-filter_roll;////fAngle[1]
        e_roll_f=abs(e_roll);
        u_roll=constrain(rollFuzzy.compute(e_roll_f,1,u_yaw_pre),-300,300);
        if (e_roll < 0) {
        u_roll = -u_roll;         
            } 
        //u_roll = abs(u_roll);  
        
        }
       else{
         u_roll=0;
         e_roll=0;
         e_roll_f=0;
         roll_setpoint=filter_roll;
        }
       
       if(data.ch[9]>=1500){
        e_yaw = yaw_setpoint-filter_yaw;
  //      u_yaw_Error=((-0.000006)*(pow(e_yaw,3)))+(0.084141*e_yaw);////only kp
  
//        u_yaw_Error=(0.084141*e_yaw);////only kp //////////tune this/////////////////////////
//        u_yaw_deError=0.001*yaw_dError;
//        u_yaw=u_yaw_Error+u_yaw_deError;
       // u_yaw=smc.compute(e_yaw,yaw_dError);
          u_yaw=constrain(yawFuzzy.compute(e_yaw,yaw_dError,u_yaw_pre),-3.8,3.8);
        if(u_yaw>3.8){
          u_yaw=3.8;
          }
        else if(-3.8>u_yaw){
          u_yaw=-3.8;
          }
          
        if(0>e_yaw){
          amplitude_RCM=abs(u_yaw)+amplitude_right;
          amplitude_LCM=1;
          //Serial.println("right");
          }
        else if(e_yaw>0){
          amplitude_LCM=abs(u_yaw)+amplitude_left;
          amplitude_RCM=1;
          //Serial.println("left");
          }
      }
      else{
        
        yaw_setpoint=filter_yaw;
        u_yaw=1;////////
        amplitude_RCM=1;/////
        amplitude_LCM=1;/////
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
    // Serial.print("y:");Serial.print(y);*
    // Serial.print(",\t");
    // Serial.print("servol:");Serial.print(servo_left_val);
    // Serial.print(",\t");
    // Serial.print("servo_center:");Serial.print((float)map(servo_left_val*10,servo_us_min*10,servo_us_max*10,servo_deg_min*10,servo_deg_max*10)/10);
    // Serial.print(",\n");
  
   }
  }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
