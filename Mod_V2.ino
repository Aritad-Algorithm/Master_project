// プログラム作成：九工大大竹研
//参考：2サーボ駆動型羽ばたき機のArduino CODE（2　Servo）http://kakutaclinic.life.coocan.jp/SFOsys2S.html
#include<ESP32Servo.h>  // https://github.com/madhephaestus/ESP32Servo
#include "sbus.h"       // https://github.com/bolderflight/sbus
#include <Ticker.h>
//#define PI 3.141592653589793
#include <wit_c_sdk.h>
#include <Wire.h>
#include <REG.h>
#include "BluetoothSerial.h"
//#pragma GCC diagnostic ignored "-Wunused-function"
//#pragma GCC diagnostic ignored "-Wunused-variable"

/*ESP32PWM pwm;
int freq = 50;
int resolution_bits = 8;
int ret __attribute__((unused)) = pwm.setup(freq, resolution_bits);
*/

bfs::SbusRx sbus_rx(&Serial1,36,18,true);  //Rx:22, Tx:23 反転あり
bfs::SbusData data;

Ticker timer_sin;
Ticker timer_servo;
Ticker timer_BT;
volatile int flag_square_wave = 0;    //矩形波の場合は1，Sin波の場合は1以外．


/////// 設定 /////////////////////////////////////////////////////////
//const int flag_square_wave = 0;    //矩形波の場合は1，Sin波の場合は1以外．
const int dt_sin = 3;              //sin波生成の周期 ms   dt_sin <= dt_servo にする
const int dt_servo = 5;            //サーボの命令更新周期 ms　使用するサーボに合わせる　標準は20（50Hz）
const int servo_deg_neutral = 90;  //サーボの中心角度 deg単位　使用するサーボに合わせる
const int servo_deg_min = 30;      //サーボの最小角度 deg単位　使用するサーボに合わせる
const int servo_deg_max = 150;     //サーボの最大角度 deg単位　使用するサーボに合わせる
const int servo_us_neutral = 1500; //サーボの中心角度 マイクロ秒単位　使用するサーボに合わせる
const int servo_us_min = 900;      //サーボの最小角度 マイクロ秒単位　使用するサーボに合わせる
const int servo_us_max = 2100;     //サーボの最大角度 マイクロ秒単位　使用するサーボに合わせる
const int propo_neutral = 1024;    //プロポの中間値　Serial.printで確認する
const int propo_min = 352;         //プロポの最小値　Serial.printで確認する
const int propo_max = 1696;        //プロポの最大値　Serial.printで確認する
const int servo_left_pin = 26;     //左サーボのピン番号
const int servo_right_pin = 19;    //右サーボのピン番号
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


BluetoothSerial SerialBT;

//Example WIT

#define ACC_UPDATE    0x01
#define GYRO_UPDATE   0x02
#define ANGLE_UPDATE  0x04
#define MAG_UPDATE    0x08
#define READ_UPDATE   0x80
static char s_cDataUpdate = 0, s_cCmd = 0xff;

//static void CmdProcess(void);
static void AutoScanSensor(void);
//static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
static int32_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t*data, uint32_t length);
static int32_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t* data, uint32_t length);
static void Delayms(uint16_t ucMs);

//Example WIT

int i;
int Air_Pressure;
volatile float fAcc[3], fGyro[3], fAngle[3];

volatile float  Height, e_height=0, e_height_pre=0, de_height=0, height0=0;

volatile int flag_r = 0, flag_p = 0, flag_y = 0, flag_h=0, c_r=0, c_p=0, c_y=0;

volatile float  KP_GAIN_r = 10, KI_GAIN_r = 0, KD_GAIN_r = 0,
                KP_GAIN_p = 15,  KI_GAIN_p = 0, KD_GAIN_p = 0,
                KP_GAIN_y = 20, KI_GAIN_y = 0, KD_GAIN_y = 0,
                KP_GAIN_h = 65, KI_GAIN_h = 0, KD_GAIN_h = 0;

volatile float  e_roll   = 0, de_roll  = 0, u_roll  = 0,
                e_pitch  = 0, de_pitch = 0, u_pitch = 0,
                e_yaw    = 0, de_yaw   = 0, u_yaw   = 0,
                u_height = 0;

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

float filter_angleZ;
float filter_gyroZ;

float filter_angleX;
float filter_gyroX;

float Pitch_Setpoint=0;
int Error_Pitch;
float Pitch_controller_Input;
float Pitch_controller_Output;
int Member_Pitch;


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
int getMember(int value) {
    if (value >= -90 && value <= -86) return 1;
    else if (value >= -85 && value <= -53) return 2;
    else if (value >= -53 && value <= -44) return (value + 56); // Mapping (-53 to -44) to (3 to 11)
    else if (value >= -43 && value <= -21) return 12;
    else if (value >= -20 && value <= -12) return (value + 33); // Mapping (-20 to -12) to (13 to 21)
    else if (value >= -11 && value <= 11) return 22;
    else if (value >= 12 && value <= 20) return (value - 11 + 22); // Mapping (12 to 20) to (23 to 31)
    else if (value >= 21 && value <= 43) return 32;
    else if (value >= 44 && value <= 52) return (value - 44 + 33); // Mapping (44 to 52) to (33 to 41)
    else if (value >= 53 && value <= 85) return 42;
    else if (value >= 86 && value <= 90) return 43;
    else if (value >= -90 && value <= -86) return 1; // Out of range
}
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
  if (data.ch[4] >= 1500) {
      flag_p = 1;//144, 1904 or 352, 1024, 1696
    }else if(data.ch[4] <= 400){
      sum_p = 0;
      c_p = 0;
    }else{
      flag_p = 0;
      sum_p += fAngle[0];
      c_p += 1;
    }
    
    if (data.ch[8] >= 1500) {
      flag_r = 1;//144, 1904 or 352, 1024, 1696
    }else if(data.ch[8] <= 400){
      roll = fAngle[1];
    }else{
      flag_r = 0;
    }

    if (data.ch[9] >= 1500) {
      flag_y = 1;//144, 1904 or 352, 1024, 1696
    }else if(data.ch[9] <= 400){
      sum_y = 0;
      c_y = 0;
    }else{
      flag_y = 0;
      sum_y += fAngle[2];
      c_y += 1;
    }

    if (data.ch[10] == 352) {
      height0 = Height;//144, 1904 or 352, 1024, 1696
      flag_h = 0;
    }else if(data.ch[10] == 1024){
      flag_h = 0;
    }else{
      flag_h = 1;
    }

   /* e_pitch = - sum_p/c_p + fAngle[0];//138;
    de_pitch = fGyro[0];
    u_pitch = KP_GAIN_p*e_pitch + KD_GAIN_p*de_pitch;*/
   
    
    e_pitch = - sum_p/c_p + fAngle[0];//138; old code setpoint
    
    Pitch_controller_Input=getMember(mov_avg(fAngle[0]));
    de_pitch = fGyro[0];
    u_pitch = (16.36*Pitch_controller_Input)-360; ////From fuzzy
    
    /*if (flag_p == 0) {
      u_pitch = 0;
    }*/

  servo_left_val = constrain(servo_left_val, servo_us_min, servo_us_max);
  servo_right_val = constrain(servo_right_val, servo_us_min, servo_us_max);

  servo_left_val  = (int)(servo_us_neutral+(-elevator-u_pitch+aileron+servo_adjust_left )+( throttle+rudder)*y);
  servo_right_val = (int)(servo_us_neutral+( elevator+u_pitch+aileron+servo_adjust_right)+(-throttle+rudder)*y);
  
  servo_left.writeMicroseconds(servo_left_val);
  servo_right.writeMicroseconds(servo_right_val);
}
void on_timer_BT(){
  SerialBT.print(millis());//マイコン起動から何秒経ったか
  SerialBT.print(",");
  SerialBT.print(Height - height0, 3);//基準点からの高度
  SerialBT.print(",");
  SerialBT.print(fAngle[1], 3);
  SerialBT.print(",");
  SerialBT.print(fAngle[0], 3);
  SerialBT.print(",");
  SerialBT.print(fAngle[2], 3);
  SerialBT.print("\r\n");
  
}

//Example WIT

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);

//Example WIT

  sbus_rx.Begin();
  SerialBT.begin("ESP32");

  servo_left.setPeriodHertz(servo_period);
  servo_right.setPeriodHertz(servo_period);
 
  servo_left.attach(servo_left_pin,servo_us_min,servo_us_max);
  servo_right.attach(servo_right_pin,servo_us_min,servo_us_max);

  delay(2000);
  timer_sin.attach_ms(dt_sin,on_timer_sin);
  timer_servo.attach_ms(dt_servo,on_timer_servo);
  timer_BT.attach_ms(5,on_timer_BT);

//Example WIT

  WitInit(WIT_PROTOCOL_I2C, 0x50);
  WitI2cFuncRegister(IICwriteBytes, IICreadBytes);
  WitRegisterCallBack(CopeSensorData);
  WitDelayMsRegister(Delayms);
  Serial.print("\r\n********************** wit-motion IIC example  ************************\r\n");
  AutoScanSensor();
  if(WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK) Serial.print("\r\nSet Bandwidth Error\r\n");
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
   
    if(s_cDataUpdate & ANGLE_UPDATE)
    {
      filter_angleY=mov_avg(fAngle[0]);
      Serial.print("angle:");
      Serial.print(fAngle[0]);
      Serial.print(",");
      Serial.print(u_pitch);
      Serial.print(",");
      Serial.print(e_pitch);
      Serial.print(",");
      Serial.print(servo_left_val);
      Serial.print(",");
      Serial.print(servo_right_val);
      Serial.print("\r\n");
      s_cDataUpdate &= ~ANGLE_UPDATE;
    }
   
    s_cDataUpdate = 0;
  }

//Example WIT

  if (sbus_rx.Read()) {
    //Serial.print(millis());
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
    
    //Serial.println(millis());
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
  //Serial.println(millis());
}

//Example WIT

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
        return ;
      }
      iRetry--;
    }while(iRetry);
  }
  Serial.print("No sensor found\r\n");
}
