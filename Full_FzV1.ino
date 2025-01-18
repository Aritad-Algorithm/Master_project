
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

#include <Wire.h>
#include <REG.h>
#include <wit_c_sdk.h>
#include "fis_header.h"

#define ACC_UPDATE    0x01
#define GYRO_UPDATE   0x02
#define ANGLE_UPDATE  0x04
#define MAG_UPDATE    0x08
#define READ_UPDATE   0x80
static char s_cDataUpdate = 0, s_cCmd = 0xff;


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
////////////////////////////////////
volatile float u_pitch = 0;
////////////////////////////////////
static void CmdProcess(void);
static void AutoScanSensor(void);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
static int32_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length);
static int32_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t* data, uint32_t length);
static void Delayms(uint16_t ucMs);

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


int i;
float fAcc[3], fGyro[3], fAngle[3];
//////////fuzzy
// Number of inputs to the fuzzy inference system
const int fis_gcI = 1;
// Number of outputs to the fuzzy inference system
const int fis_gcO = 1;
// Number of rules to the fuzzy inference system
const int fis_gcR = 5;

FIS_TYPE g_fisInput[fis_gcI];
FIS_TYPE g_fisOutput[fis_gcO];

// Count of member function for each Input
int fis_gIMFCount[] = { 5 };

// Count of member function for each Output 
int fis_gOMFCount[] = { 5 };


// Input membership function set
int fis_gMFI0[] = { 0, 0, 0, 0, 0 };
int* fis_gMFI[] = { fis_gMFI0};

// Output membership function set
int fis_gMFO0[] = { 0, 0, 0, 0, 0 };
int* fis_gMFO[] = { fis_gMFO0};

// Rule Weights
FIS_TYPE fis_gRWeight[] = { 1, 1, 1, 1, 1 };

// Rule Type
int fis_gRType[] = { 1, 1, 1, 1, 1 };

// Rule Inputs
int fis_gRI0[] = { 1 };
int fis_gRI1[] = { 2 };
int fis_gRI2[] = { 3 };
int fis_gRI3[] = { 4 };
int fis_gRI4[] = { 5 };
int* fis_gRI[] = { fis_gRI0, fis_gRI1, fis_gRI2, fis_gRI3, fis_gRI4 };

// Rule Outputs
int fis_gRO0[] = { 1 };
int fis_gRO1[] = { 2 };
int fis_gRO2[] = { 3 };
int fis_gRO3[] = { 4 };
int fis_gRO4[] = { 5 };
int* fis_gRO[] = { fis_gRO0, fis_gRO1, fis_gRO2, fis_gRO3, fis_gRO4 };

// Input range Min
FIS_TYPE fis_gIMin[] = { -85 };

// Input range Max
FIS_TYPE fis_gIMax[] = { 85 };

// Output range Min
FIS_TYPE fis_gOMin[] = { -400 };

// Output range Max
FIS_TYPE fis_gOMax[] = { 400 };

//***********************************************************************
// Data for Fuzzy Inference System                                       
//***********************************************************************
// Coefficients for the Input Member Functions
FIS_TYPE fis_gMFI0Coeff1[] = { 43, 53, 85, 85 };
FIS_TYPE fis_gMFI0Coeff2[] = { 11, 21, 43, 53 };
FIS_TYPE fis_gMFI0Coeff3[] = { -21, -11, 11, 21 };
FIS_TYPE fis_gMFI0Coeff4[] = { -53, -43, -21, -11 };
FIS_TYPE fis_gMFI0Coeff5[] = { -85, -85, -53, -43 };
FIS_TYPE* fis_gMFI0Coeff[] = { fis_gMFI0Coeff1, fis_gMFI0Coeff2, fis_gMFI0Coeff3, fis_gMFI0Coeff4, fis_gMFI0Coeff5 };
FIS_TYPE** fis_gMFICoeff[] = { fis_gMFI0Coeff };

// Coefficients for the Output Member Functions
FIS_TYPE fis_gMFO0Coeff1[] = { 220, 260, 400, 400 };
FIS_TYPE fis_gMFO0Coeff2[] = { 60, 100, 220, 260 };
FIS_TYPE fis_gMFO0Coeff3[] = { -100, -60, 60, 100 };
FIS_TYPE fis_gMFO0Coeff4[] = { -260, -220, -100, -60 };
FIS_TYPE fis_gMFO0Coeff5[] = { -400, -400, -260, -220 };
FIS_TYPE* fis_gMFO0Coeff[] = { fis_gMFO0Coeff1, fis_gMFO0Coeff2, fis_gMFO0Coeff3, fis_gMFO0Coeff4, fis_gMFO0Coeff5 };
FIS_TYPE** fis_gMFOCoeff[] = { fis_gMFO0Coeff };
//***********************************************************************
// Data dependent support functions for Fuzzy Inference System           
//***********************************************************************
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
////////////////////////////////////////////////////// Add controller Here//////////////////////////////////////
void on_timer_servo(){
  u_pitch=g_fisOutput[0];
  servo_left_val =  (int)(servo_us_neutral+(-elevator-u_pitch+aileron+servo_adjust_left)+(throttle+rudder)*y);
  servo_right_val = (int)(servo_us_neutral+(elevator+u_pitch+aileron+servo_adjust_right)+(-throttle+rudder)*y);

  servo_left.writeMicroseconds(servo_left_val);
  servo_right.writeMicroseconds(servo_right_val);
}
/////////////////////////////////////////////////////
void setup()
{
  Serial.begin(115200);
  //feedback system
  Wire.begin();
  Wire.setClock(400000);
  ////
  sbus_rx.Begin();

  servo_left.setPeriodHertz(servo_period);
  servo_right.setPeriodHertz(servo_period);
 
  servo_left.attach(servo_left_pin,servo_us_min,servo_us_max);
  servo_right.attach(servo_right_pin,servo_us_min,servo_us_max);

  delay(2000);
  timer_sin.attach_ms(dt_sin,on_timer_sin);
  timer_servo.attach_ms(dt_servo,on_timer_servo);
  ////
  WitInit(WIT_PROTOCOL_I2C, 0x50);
  WitI2cFuncRegister(IICwriteBytes, IICreadBytes);
  WitRegisterCallBack(CopeSensorData);
  WitDelayMsRegister(Delayms);
  Serial.print("\r\n********************** wit-motion IIC example  ************************\r\n");
  AutoScanSensor();
}
void loop()
{
feedback();
Filter_feedback();
Serial_sent();
Controller();
delayMicroseconds(0.5);  
}
void Controller(){
  g_fisInput[0] = filter_angleY;
  // Initialize the output
  g_fisOutput[0] = 0;
  // Evaluate the fuzzy inference system
  fis_evaluate();
  // Print the output value
  Serial.print("Input : ");
  Serial.print(filter_angleY);
  Serial.print(" -> Output Pitch: ");
  Serial.println(g_fisOutput[0]);
  
  }
void Serial_sent(){
  Serial.print(servo_left_val);
  Serial.print(",");
  Serial.println(servo_right_val);  
  }
void Filter_feedback(){
  filter_angleY=mov_avg(fAngle[0]);
  filter_gyroY=kalmanUpdate(fGyro[0]);
  
  filter_angleX=mov_avg(fAngle[1]);
  filter_gyroX=kalmanUpdate(fGyro[2]);

  filter_angleZ=mov_avg(fAngle[2]);
  filter_gyroZ=kalmanUpdate(fGyro[1]);
  
  }

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

  
void feedback() {
    WitReadReg(AX, 21);
    delay(500);
    while (Serial.available()) 
    {
      CopeCmdData(Serial.read());
    }
    CmdProcess();
    if(s_cDataUpdate)
    {
      for(i = 0; i < 3; i++)
      {
        fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
        fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
        fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
      }
      
      s_cDataUpdate = 0;
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

static void ShowHelp(void)
{
  Serial.print("\r\n************************   WIT_SDK_DEMO ************************");
  Serial.print("\r\n************************          HELP           ************************\r\n");
  Serial.print("UART SEND:a\\r\\n   Acceleration calibration.\r\n");
  Serial.print("UART SEND:m\\r\\n   Magnetic field calibration,After calibration send:   e\\r\\n   to indicate the end\r\n");
  Serial.print("UART SEND:U\\r\\n   Bandwidth increase.\r\n");
  Serial.print("UART SEND:u\\r\\n   Bandwidth reduction.\r\n");
  Serial.print("UART SEND:B\\r\\n   Baud rate increased to 115200.\r\n");
  Serial.print("UART SEND:b\\r\\n   Baud rate reduction to 9600.\r\n");
  Serial.print("UART SEND:h\\r\\n   help.\r\n");
  Serial.print("******************************************************************************\r\n");
}

static void CmdProcess(void)
{
  switch(s_cCmd)
  {
    case 'a': if(WitStartAccCali() != WIT_HAL_OK) Serial.print("\r\nSet AccCali Error\r\n");
      break;
    case 'm': if(WitStartMagCali() != WIT_HAL_OK) Serial.print("\r\nSet MagCali Error\r\n");
      break;
    case 'e': if(WitStopMagCali() != WIT_HAL_OK) Serial.print("\r\nSet MagCali Error\r\n");
      break;
    case 'u': if(WitSetBandwidth(BANDWIDTH_5HZ) != WIT_HAL_OK) Serial.print("\r\nSet Bandwidth Error\r\n");
      break;
    case 'U': if(WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK) Serial.print("\r\nSet Bandwidth Error\r\n");
      break;
    case 'B': if(WitSetUartBaud(WIT_BAUD_115200) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
              else Serial.print(" 115200 Baud rate modified successfully\r\n");
      break;
    case 'b': if(WitSetUartBaud(WIT_BAUD_9600) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
              else Serial.print(" 9600 Baud rate modified successfully\r\n");
      break;
    case 'h': ShowHelp();
      break;
    default :return;
  }
  s_cCmd = 0xff;
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
        ShowHelp();
        return ;
      }
      iRetry--;
    }while(iRetry);   
  }
  Serial.print("can not find sensor\r\n");
  Serial.print("please check your connection\r\n");
}

//***********************************************************************
// Support functions for Fuzzy Inference System                          
//***********************************************************************
// Trapezoidal Member Function
FIS_TYPE fis_trapmf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE a = p[0], b = p[1], c = p[2], d = p[3];
    FIS_TYPE t1 = ((x <= c) ? 1 : ((d < x) ? 0 : ((c != d) ? ((d - x) / (d - c)) : 0)));
    FIS_TYPE t2 = ((b <= x) ? 1 : ((x < a) ? 0 : ((a != b) ? ((x - a) / (b - a)) : 0)));
    return (FIS_TYPE) min(t1, t2);
}

FIS_TYPE fis_min(FIS_TYPE a, FIS_TYPE b)
{
    return min(a, b);
}

FIS_TYPE fis_max(FIS_TYPE a, FIS_TYPE b)
{
    return max(a, b);
}

FIS_TYPE fis_array_operation(FIS_TYPE *array, int size, _FIS_ARR_OP pfnOp)
{
    int i;
    FIS_TYPE ret = 0;

    if (size == 0) return ret;
    if (size == 1) return array[0];

    ret = array[0];
    for (i = 1; i < size; i++)
    {
        ret = (*pfnOp)(ret, array[i]);
    }

    return ret;
}
// Pointers to the implementations of member functions
_FIS_MF fis_gMF[] =
{
    fis_trapmf
};

FIS_TYPE fis_MF_out(FIS_TYPE** fuzzyRuleSet, FIS_TYPE x, int o)
{
    FIS_TYPE mfOut;
    int r;

    for (r = 0; r < fis_gcR; ++r)
    {
        int index = fis_gRO[r][o];
        if (index > 0)
        {
            index = index - 1;
            mfOut = (fis_gMF[fis_gMFO[o][index]])(x, fis_gMFOCoeff[o][index]);
        }
        else if (index < 0)
        {
            index = -index - 1;
            mfOut = 1 - (fis_gMF[fis_gMFO[o][index]])(x, fis_gMFOCoeff[o][index]);
        }
        else
        {
            mfOut = 0;
        }

        fuzzyRuleSet[0][r] = fis_min(mfOut, fuzzyRuleSet[1][r]);
    }
    return fis_array_operation(fuzzyRuleSet[0], fis_gcR, fis_max);
}

FIS_TYPE fis_defuzz_centroid(FIS_TYPE** fuzzyRuleSet, int o)
{
    FIS_TYPE step = (fis_gOMax[o] - fis_gOMin[o]) / (FIS_RESOLUSION - 1);
    FIS_TYPE area = 0;
    FIS_TYPE momentum = 0;
    FIS_TYPE dist, slice;
    int i;

    // calculate the area under the curve formed by the MF outputs
    for (i = 0; i < FIS_RESOLUSION; ++i){
        dist = fis_gOMin[o] + (step * i);
        slice = step * fis_MF_out(fuzzyRuleSet, dist, o);
        area += slice;
        momentum += slice*dist;
    }

    return ((area == 0) ? ((fis_gOMax[o] + fis_gOMin[o]) / 2) : (momentum / area));
}

//***********************************************************************
// Fuzzy Inference System                                                
//***********************************************************************
void fis_evaluate()
{
    FIS_TYPE fuzzyInput0[] = { 0, 0, 0, 0, 0 };
    FIS_TYPE* fuzzyInput[fis_gcI] = { fuzzyInput0, };
    FIS_TYPE fuzzyOutput0[] = { 0, 0, 0, 0, 0 };
    FIS_TYPE* fuzzyOutput[fis_gcO] = { fuzzyOutput0, };
    FIS_TYPE fuzzyRules[fis_gcR] = { 0 };
    FIS_TYPE fuzzyFires[fis_gcR] = { 0 };
    FIS_TYPE* fuzzyRuleSet[] = { fuzzyRules, fuzzyFires };
    FIS_TYPE sW = 0;

    // Transforming input to fuzzy Input
    int i, j, r, o;
    for (i = 0; i < fis_gcI; ++i)
    {
        for (j = 0; j < fis_gIMFCount[i]; ++j)
        {
            fuzzyInput[i][j] =
                (fis_gMF[fis_gMFI[i][j]])(g_fisInput[i], fis_gMFICoeff[i][j]);
        }
    }

    int index = 0;
    for (r = 0; r < fis_gcR; ++r)
    {
        if (fis_gRType[r] == 1)
        {
            fuzzyFires[r] = FIS_MAX;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_min(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_min(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_min(fuzzyFires[r], 1);
            }
        }
        else
        {
            fuzzyFires[r] = FIS_MIN;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_max(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_max(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_max(fuzzyFires[r], 0);
            }
        }

        fuzzyFires[r] = fis_gRWeight[r] * fuzzyFires[r];
        sW += fuzzyFires[r];
    }

    if (sW == 0)
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            g_fisOutput[o] = ((fis_gOMax[o] + fis_gOMin[o]) / 2);
        }
    }
    else
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            g_fisOutput[o] = fis_defuzz_centroid(fuzzyRuleSet, o);
        }
    }
}
