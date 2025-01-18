#include <Wire.h>
#include <REG.h>
#include <wit_c_sdk.h>
/*
Test on MEGA 2560. use JY901S sensor

 JY901S           MEGA 2560
  VCC      <--->   5V/3.3V
  SCL      <--->   SCL(21)
  SDA      <--->   SDA(20)
  GND      <--->    GND
*/
#define ACC_UPDATE    0x01
#define GYRO_UPDATE   0x02
#define ANGLE_UPDATE  0x04
#define MAG_UPDATE    0x08
#define READ_UPDATE   0x80
static char s_cDataUpdate = 0, s_cCmd = 0xff;

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
void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  Wire.begin();
  Wire.setClock(400000);
  WitInit(WIT_PROTOCOL_I2C, 0x50);
  WitI2cFuncRegister(IICwriteBytes, IICreadBytes);
  WitRegisterCallBack(CopeSensorData);
  WitDelayMsRegister(Delayms);
  Serial.print("\r\n********************** wit-motion IIC example  ************************\r\n");
  AutoScanSensor();
}

void loop(){
  feedback();
  Filter_feedback();
  Serial_sent();
  delayMicroseconds(1);
  }
void Serial_sent(){
  Serial.print(filter_angleX); //Serial sent
  Serial.print(",");
  Serial.print(filter_angleY);
  Serial.print(",");
  Serial.print(filter_angleZ);
  Serial.print(",");
  Serial.print(filter_gyroX);
  Serial.print(",");
  Serial.print(filter_gyroY);
  Serial.print(",");
  Serial.println(filter_gyroZ); 
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
