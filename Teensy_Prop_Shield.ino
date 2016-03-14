
/* Teensy Prop Shield Example Code
 Original sketch by: Kris Winer
 with pieces borrowed from Jim Linblom of sparkfun.com
 date: May 31, 2014
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 Includes reset, initialization, accelerometer calibration, sleep mode, motion threshold, portrait/lanscape detection
 and tap detection function. Tried to get the acceleration magnitude detection to work but it seems to be always on!
 
 This code provides example usage for most features of
 the FXOS8700CQ 3-axis, I2C 14-bit accelerometer/16-bit magnetometer. 
 Play around with the settings in the various functions and consult the data sheet.

 FXAS21000 is a small, low-power, 3-axis yaw, pitch, and roll
 angular rate gyroscope. The full-scale range is adjustable from
 ±200°/s to ±1600°/s. It features both I2C and SPI interfaces. We parameterize the registers, 
 calibrate the gyro, get properly scaled angular rates, and display all on an on-breadboard.

 IDE example usage for most features of the MPL3115A2 I2C Precision Altimeter.
 
 Basic functions are implemented including absolute pressure (50 to 110 kPa), altimeter pressure (mmHg), 
 altitude (meters or feet), and temperature (-40 to 85 C).
 
 In addition, provision for the FIFO mode in the initialization, watermark/overflow setting, and register
 read functions for autonomous data logging over as many as nine hours with 32 data samples of P, T.

 */
 
#include "i2c_t3.h"  
#include <SPI.h>

// Define registers per FXOS8700CQ, Document Number: FXOS8700CQ 
// http://cache.freescale.com/files/sensors/doc/data_sheet/FXOS8700CQ.pdf
// Data Sheet: Technical Data Rev. 2.0, 02/2013 3-Axis, 12-bit/8-bit Digital Accelerometer
// Freescale Semiconductor Data Sheet
#define FXOS8700CQ_STATUS           0x00
#define FXOS8700CQ_DR_STATUS        0x00
#define FXOS8700CQ_F_STATUS         0x00
#define FXOS8700CQ_OUT_X_MSB        0x01    
#define FXOS8700CQ_OUT_X_LSB        0x02
#define FXOS8700CQ_OUT_Y_MSB        0x03
#define FXOS8700CQ_OUT_Y_LSB        0x04
#define FXOS8700CQ_OUT_Z_MSB        0x05
#define FXOS8700CQ_OUT_Z_LSB        0x06
#define FXOS8700CQ_F_SETUP          0x09
#define FXOS8700CQ_TRIG_CFG         0x0A
#define FXOS8700CQ_SYSMOD           0x0B
#define FXOS8700CQ_INT_SOURCE       0x0C
#define FXOS8700CQ_WHO_AM_I         0x0D   
#define FXOS8700CQ_XYZ_DATA_CFG     0x0E
#define FXOS8700CQ_HP_FILTER_CUTOFF 0x0F
#define FXOS8700CQ_PL_STATUS        0x10
#define FXOS8700CQ_PL_CFG           0x11
#define FXOS8700CQ_PL_COUNT         0x12
#define FXOS8700CQ_PL_BF_ZCOMP      0x13
#define FXOS8700CQ_P_L_THS_REG      0x14
#define FXOS8700CQ_A_FFMT_CFG       0x15
#define FXOS8700CQ_A_FFMT_SRC       0x16
#define FXOS8700CQ_A_FFMT_THS       0x17
#define FXOS8700CQ_A_FFMT_COUNT     0x18
#define FXOS8700CQ_TRANSIENT_CFG    0x1D
#define FXOS8700CQ_TRANSIENT_SRC    0x1E
#define FXOS8700CQ_TRANSIENT_THS    0x1F
#define FXOS8700CQ_TRANSIENT_COUNT  0x20
#define FXOS8700CQ_PULSE_CFG        0x21
#define FXOS8700CQ_PULSE_SRC        0x22
#define FXOS8700CQ_PULSE_THSX       0x23
#define FXOS8700CQ_PULSE_THSY       0x24
#define FXOS8700CQ_PULSE_THSZ       0x25
#define FXOS8700CQ_PULSE_TMLT       0x26
#define FXOS8700CQ_PULSE_LTCY       0x27
#define FXOS8700CQ_PULSE_WIND       0x28
#define FXOS8700CQ_ASLP_COUNT       0x29
#define FXOS8700CQ_CTRL_REG1        0x2A
#define FXOS8700CQ_CTRL_REG2        0x2B
#define FXOS8700CQ_CTRL_REG3        0x2C
#define FXOS8700CQ_CTRL_REG4        0x2D
#define FXOS8700CQ_CTRL_REG5        0x2E
#define FXOS8700CQ_A_OFF_X          0x2F
#define FXOS8700CQ_A_OFF_Y          0x30
#define FXOS8700CQ_A_OFF_Z          0x31
#define FXOS8700CQ_M_DR_STATUS      0x32
#define FXOS8700CQ_M_OUT_X_MSB      0x33    
#define FXOS8700CQ_M_OUT_X_LSB      0x34
#define FXOS8700CQ_M_OUT_Y_MSB      0x35
#define FXOS8700CQ_M_OUT_Y_LSB      0x36
#define FXOS8700CQ_M_OUT_Z_MSB      0x37
#define FXOS8700CQ_M_OUT_Z_LSB      0x38
#define FXOS8700CQ_CMP_OUT_X_MSB    0x39    
#define FXOS8700CQ_CMP_OUT_X_LSB    0x3A
#define FXOS8700CQ_CMP_OUT_Y_MSB    0x3B
#define FXOS8700CQ_CMP_OUT_Y_LSB    0x3C
#define FXOS8700CQ_CMP_OUT_Z_MSB    0x3D
#define FXOS8700CQ_CMP_OUT_Z_LSB    0x3E
#define FXOS8700CQ_M_OFF_X_MSB      0x3F    
#define FXOS8700CQ_M_OFF_X_LSB      0x40
#define FXOS8700CQ_M_OFF_Y_MSB      0x41
#define FXOS8700CQ_M_OFF_Y_LSB      0x42
#define FXOS8700CQ_M_OFF_Z_MSB      0x43
#define FXOS8700CQ_M_OFF_Z_LSB      0x44
#define FXOS8700CQ_MAX_X_MSB        0x45   
#define FXOS8700CQ_MAX_X_LSB        0x46
#define FXOS8700CQ_MAX_Y_MSB        0x47
#define FXOS8700CQ_MAX_Y_LSB        0x48
#define FXOS8700CQ_MAX_Z_MSB        0x49
#define FXOS8700CQ_MAX_Z_LSB        0x4A
#define FXOS8700CQ_MIN_X_MSB        0x4B   
#define FXOS8700CQ_MIN_X_LSB        0x4C
#define FXOS8700CQ_MIN_Y_MSB        0x4D
#define FXOS8700CQ_MIN_Y_LSB        0x4E
#define FXOS8700CQ_MIN_Z_MSB        0x4F
#define FXOS8700CQ_MIN_Z_LSB        0x50
#define FXOS8700CQ_TEMP             0x51
#define FXOS8700CQ_M_THS_CFG        0x52
#define FXOS8700CQ_M_THS_SRC        0x53
#define FXOS8700CQ_M_THS_X_MSB      0x54   
#define FXOS8700CQ_M_THS_X_LSB      0x55
#define FXOS8700CQ_M_THS_Y_MSB      0x56
#define FXOS8700CQ_M_THS_Y_LSB      0x57
#define FXOS8700CQ_M_THS_Z_MSB      0x58
#define FXOS8700CQ_M_THS_Z_LSB      0x59
#define FXOS8700CQ_M_THS_COUNT      0x5A
#define FXOS8700CQ_M_CTRL_REG1      0x5B
#define FXOS8700CQ_M_CTRL_REG2      0x5C
#define FXOS8700CQ_M_CTRL_REG3      0x5D
#define FXOS8700CQ_M_INT_SRC        0x5E
#define FXOS8700CQ_A_VECM_CFG       0x5F
#define FXOS8700CQ_A_VECM_THS_MSB   0x60
#define FXOS8700CQ_A_VECM_THS_LSB   0x61
#define FXOS8700CQ_A_VECM_CNT       0x62
#define FXOS8700CQ_A_VECM_INITX_MSB 0x63   
#define FXOS8700CQ_A_VECM_INITX_LSB 0x64
#define FXOS8700CQ_A_VECM_INITY_MSB 0x65
#define FXOS8700CQ_A_VECM_INITY_LSB 0x66
#define FXOS8700CQ_A_VECM_INITZ_MSB 0x67
#define FXOS8700CQ_A_VECM_INITZ_LSB 0x68
#define FXOS8700CQ_M_VECM_CFG       0x69
#define FXOS8700CQ_M_VECM_THS_MSB   0x6A
#define FXOS8700CQ_M_VECM_THS_LSB   0x6B
#define FXOS8700CQ_M_VECM_CNT       0x6C
#define FXOS8700CQ_M_VECM_INITX_MSB 0x6D   
#define FXOS8700CQ_M_VECM_INITX_LSB 0x6E
#define FXOS8700CQ_M_VECM_INITY_MSB 0x6F
#define FXOS8700CQ_M_VECM_INITY_LSB 0x70
#define FXOS8700CQ_M_VECM_INITZ_MSB 0x71
#define FXOS8700CQ_M_VECM_INITZ_LSB 0x72
#define FXOS8700CQ_A_FFMT_THS_X_MSB 0x73
#define FXOS8700CQ_A_FFMT_THS_X_LSB 0x74
#define FXOS8700CQ_A_FFMT_THS_Y_MSB 0x75
#define FXOS8700CQ_A_FFMT_THS_Y_LSB 0x76
#define FXOS8700CQ_A_FFMT_THS_Z_MSB 0x77
#define FXOS8700CQ_A_FFMT_THS_Z_LSB 0x78

// Define registers per Freescale Semiconductor, Inc.
// FXAS21000 Data Sheet: Advance Information Rev 1.1, 10/2013 3-Axis, 14-bit Digital MEMS Gyroscope
// http://cache.freescale.com/files/sensors/doc/data_sheet/FXAS21000.pdf
// Freescale Semiconductor Data Sheet
#define FXAS21000_STATUS           0x00
#define FXAS21000_OUT_X_MSB        0x01    
#define FXAS21000_OUT_X_LSB        0x02
#define FXAS21000_OUT_Y_MSB        0x03
#define FXAS21000_OUT_Y_LSB        0x04
#define FXAS21000_OUT_Z_MSB        0x05
#define FXAS21000_OUT_Z_LSB        0x06
#define FXAS21000_DR_STATUS        0x07
#define FXAS21000_F_STATUS         0x08
#define FXAS21000_F_EVENT          0x0A
#define FXAS21000_INT_SRC_FLAG     0x0B
#define FXAS21000_WHO_AM_I         0x0C   // Should return 0xD7
#define FXAS21000_CTRL_REG0        0x0D
#define FXAS21000_RT_CFG           0x0E   
#define FXAS21000_RT_SRC           0x0F
#define FXAS21000_RT_THS           0x10
#define FXAS21000_RT_COUNT         0x11
#define FXAS21000_TEMP             0x12
#define FXAS21000_CTRL_REG1        0x13
#define FXAS21000_CTRL_REG2        0x14

// Freescale Semiconductor MPL3115A2 Precision I2C altimeter 
// http://www.nxp.com/files/sensors/doc/data_sheet/MPL3115A2.pdf
// Register defines courtesy A. Weiss and Nathan Seidle, SparkFun Electronics
#define MPL3115A2_STATUS           0x00
#define MPL3115A2_OUT_P_MSB        0x01
#define MPL3115A2_OUT_P_CSB        0x02
#define MPL3115A2_OUT_P_LSB        0x03
#define MPL3115A2_OUT_T_MSB        0x04
#define MPL3115A2_OUT_T_LSB        0x05
#define MPL3115A2_DR_STATUS        0x06
#define MPL3115A2_OUT_P_DELTA_MSB  0x07
#define MPL3115A2_OUT_P_DELTA_CSB  0x08
#define MPL3115A2_OUT_P_DELTA_LSB  0x09
#define MPL3115A2_OUT_T_DELTA_MSB  0x0A
#define MPL3115A2_OUT_T_DELTA_LSB  0x0B
#define MPL3115A2_WHO_AM_I         0x0C
#define MPL3115A2_F_STATUS         0x0D
#define MPL3115A2_F_DATA           0x0E
#define MPL3115A2_F_SETUP          0x0F
#define MPL3115A2_TIME_DLY         0x10
#define MPL3115A2_SYSMOD           0x11
#define MPL3115A2_INT_SOURCE       0x12
#define MPL3115A2_PT_DATA_CFG      0x13
#define MPL3115A2_BAR_IN_MSB       0x14 // Set at factory to equivalent sea level pressure for measurement location, generally no need to change
#define MPL3115A2_BAR_IN_LSB       0x15 // Set at factory to equivalent sea level pressure for measurement location, generally no need to change
#define MPL3115A2_P_TGT_MSB        0x16
#define MPL3115A2_P_TGT_LSB        0x17
#define MPL3115A2_T_TGT            0x18
#define MPL3115A2_P_WND_MSB        0x19
#define MPL3115A2_P_WND_LSB        0x1A
#define MPL3115A2_T_WND            0x1B
#define MPL3115A2_P_MIN_MSB        0x1C
#define MPL3115A2_P_MIN_CSB        0x1D
#define MPL3115A2_P_MIN_LSB        0x1E
#define MPL3115A2_T_MIN_MSB        0x1F
#define MPL3115A2_T_MIN_LSB        0x20
#define MPL3115A2_P_MAX_MSB        0x21
#define MPL3115A2_P_MAX_CSB        0x22
#define MPL3115A2_P_MAX_LSB        0x23
#define MPL3115A2_T_MAX_MSB        0x24
#define MPL3115A2_T_MAX_LSB        0x25
#define MPL3115A2_CTRL_REG1        0x26
#define MPL3115A2_CTRL_REG2        0x27
#define MPL3115A2_CTRL_REG3        0x28
#define MPL3115A2_CTRL_REG4        0x29
#define MPL3115A2_CTRL_REG5        0x2A
#define MPL3115A2_OFF_P            0x2B
#define MPL3115A2_OFF_T            0x2C
#define MPL3115A2_OFF_H            0x2D

#define FXOS8700CQ_ADDRESS 0x1E
#define FXAS21000_ADDRESS  0x20
#define MPL3115A2_ADDRESS  0x60   

// Set initial input parameters
enum accelFSR {
  AFS_2g = 0,
  AFS_4g,
  AFS_8g
};

enum accelODR {
  AODR_800HZ = 0, // 200 Hz
  AODR_400HZ,
  AODR_200HZ,
  AODR_100HZ,
  AODR_50HZ,
  AODR_12_5HZ, // 12.5 Hz, etc.
  AODR_6_25HZ,
  AODR_1_56HZ
};

enum magOSR {
  MOSR_0 = 0,  // oversample ratio 2 at 50 and 200 Hz ODR
  MOSR_1,
  MOSR_2,
  MOSR_3,
  MOSR_4,
  MOSR_5,  
  MOSR_6,
  MOSR_7      // oversample ratio 8 at 200 Hz ODR, 32 at 50 HZ ODR
};

enum SAMPLERATE {
  OS_6ms = 0, // 6 ms is minimum oversampling interval, corresponds to an oversample ration of 2^0 = 1 
  OS_10ms,
  OS_18ms,
  OS_34ms,
  OS_66ms,
  OS_130ms, // 130 ms oversampling interval, 2^5 = 32 oversample ratio
  OS_258ms,
  OS_512ms
};

enum ST_VALUE {
  ATS_1s = 0, // 6 ms is minimum oversampling interval, corresponds to an oversample ration of 2^0 = 1 
  ATS_2s,
  ATS_4s,
  ATS_8s,
  ATS_16s,
  ATS_32s,
  ATS_64s, // 2^6 = 64 s interval between up to 32 FIFO samples for half an hour of data logging
  ATS_128s,
  ATS_256s,
  ATS_512s,
  ATS_1024s,
  ATS_2048s,
  ATS_4096s,
  ATS_8192s,
  ATS_16384s,
  ATS_32768s  // 2^15 = 32768 s interval between up to 32 FIFO samples = 12 days of data logging!
};

// Set initial input parameters
enum gyroFSR {
  GFS_1600DPS = 0,
  GFS_800DPS,
  GFS_400DPS,
  GFS_200DPS
};

enum gyroODR {
  GODR_200HZ = 0, // 200 Hz
  GODR_100HZ,
  GODR_50HZ,
  GODR_25HZ,
  GODR_12_5HZ,
  GODR_6_25HZ, // 6.25 Hz, etc.
  GODR_3_125HZ,
  GODR_1_5625HZ
};

// Specify FXAS21000 gyro settings
uint8_t gyroFSR = GFS_200DPS;
uint8_t gyroODR = GODR_200HZ;
float gRes, gBias[3] = {0, 0, 0}; // scale resolutions per LSB for the sensors

// Specify MPL3115 Altimeter settings
uint8_t SAMPLERATE = OS_130ms;
uint8_t ST_VALUE   = ATS_4s;
int AltimeterMode  = 0;        // use to choose between altimeter and barometer modes for FIFO data
float temperature, pressure, altitude;

// Specify FXOS8700CQ accel/mag settings
uint8_t accelFSR = AFS_2g;     // Set the scale below either 2, 4 or 8
uint8_t accelODR = AODR_200HZ; // In hybrid mode, accel/mag data sample rates are half of this value
uint8_t   magOSR = MOSR_5;     // Choose magnetometer oversample rate
float aRes, mRes, aBias[3] = {0, 0, 0}, mBias[3] = {0, 0, 0};              // Scale resolutions per LSB for the sensor

// Pin definitions
int ledPin  = 13;  // Teensy led

int16_t magCount[3], accelCount[3], gyroCount[3];  // Stores the 12-bit signed value
float ax, ay, az;       // Stores the real accel value in g's
float mx, my, mz;       // Stores the real mag value in G's
float gx, gy, gz;  // Stores the real accel value in g's
int8_t gyrotempCount, acceltempCount;
float gyrotemperature, acceltemperature;

boolean sleepMode = false;
boolean clickMode = false;

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrtf(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate
float pitch, yaw, roll;
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval

float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

void setup()
{
  Serial.begin(38400);
  delay(5000);
  
   // Setup for Master mode, pins 16/17, external pullups, 400kHz for Teensy 3.1
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
      
   I2Cscan();
 
  // Set up the interrupt pins, they're set as active high, push-pull
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // Read the WHO_AM_I registers, this is a good test of communication
  byte c = readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_WHO_AM_I);  // Read accel/mag WHO_AM_I register
  Serial.print("FXOS8700CQ I am 0x"); Serial.print(c, HEX); Serial.println(" I should be 0xC7");
  byte d = readByte(FXAS21000_ADDRESS, FXAS21000_WHO_AM_I);  // Read gyro WHO_AM_I register
  Serial.print("FXOS8700CQ I am 0x"); Serial.print(d, HEX); Serial.println(" I should be 0xD7");
  byte e = readByte(MPL3115A2_ADDRESS, MPL3115A2_WHO_AM_I);  // Read altimeter WHO_AM_I register
  Serial.print("MPL3115A2  I am 0x"); Serial.print(e, HEX); Serial.println(" I should be 0xC4");
  delay(1000);

  if (c == 0xC7 && d == 0xD7 && e == 0xC4) // WHO_AM_I should always be 0xC7, 0xD1, and 0xC4
  {  
     // Define sensor sensitivities
     getAres();                   // get accelerometer sensitivity (g/LSB)
     mRes = 1.;                   // get magnetometer sensitivity (0.1 microTesla/LSB or 1 milliGauss/LSB)
     getGres();                   // get gyro sensitivity (dps/LSB)
 
    FXOS8700CQReset();           // Start by resetting sensor device to default settings
    calibrateFXOS8700CQ();       // Calibrate the accelerometer

    FXAS21000Reset();            // Start by resetting sensor device to default settings
    calibrateFXAS21000(gBias);   // Calculate gyro offset bias
 
    FXOS8700CQMagOffset();       // Determine magnetometer offsets-currently calculate the offsets dynamically
    
    initFXOS8700CQ();            // Initialize the accelerometer and magnetometer if communication is OK
    accelMotionIntFXOS8700CQ();  // Configure motion interrupts
    sleepModeFXOS8700CQ();       // Configure sleep mode
    Serial.println("FXOS8700CQQ is online...");
    delay (1000);

    initFXAS21000();  // init the accelerometer if communication is OK
    Serial.println("FXAS21000Q is online...");
    delay (1000);

    MPL3115A2Reset();                // Start off by resetting all registers to the default
    initRealTimeMPL3115A2();         // initialize the altimeter for realtime data acquisition if communication is OK
    MPL3115A2SampleRate(SAMPLERATE); // Set oversampling ratio
    MPL3115A2enableEventflags();     // Set data ready enable
    Serial.println("MPL3115A2 event flags enabled...");
    delay (1000);


    aBias[0] = 2*readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_OFF_X);
    aBias[1] = 2*readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_OFF_Y);
    aBias[2] = 2*readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_OFF_Z);
    Serial.println("FXOS8700CQ accelerometer calibration results");
    Serial.print("AxBias = "); Serial.print(aBias[0]); Serial.println(" mg");
    Serial.print("AyBias = "); Serial.print(aBias[1]); Serial.println(" mg");
    Serial.print("AzBias = "); Serial.print(aBias[2]); Serial.println(" mg");
    delay(1000);

    mBias[0] = (int16_t)readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OFF_X_MSB) << 8 | readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OFF_X_LSB);
    mBias[1] = (int16_t)readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OFF_Y_MSB) << 8 | readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OFF_Y_LSB);
    mBias[2] = (int16_t)readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OFF_Z_MSB) << 8 | readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OFF_Z_LSB);
    Serial.println("FXOS8700CQ magnetometer calibration results");
    Serial.print("MxBias = "); Serial.print((int16_t)(((float)mBias[0])*mRes)); Serial.println(" milliGauss");
    Serial.print("MyBias = "); Serial.print((int16_t)(((float)mBias[1])*mRes)); Serial.println(" milliGauss");
    Serial.print("MzBias = "); Serial.print((int16_t)(((float)mBias[2])*mRes)); Serial.println(" milliGauss");
    delay(1000);

    Serial.println("FXAS21000Q calibration results");
    Serial.print("GxBias = "); Serial.print(gBias[0], 2); Serial.println(" o/s");
    Serial.print("GyBias = "); Serial.print(gBias[1], 2); Serial.println(" o/s");
    Serial.print("GzBias = "); Serial.print(gBias[2], 2); Serial.println(" o/s");
    delay(1000);
   
     Serial.print("Oversampling Ratio is "); Serial.println(1<<SAMPLERATE);  

  }
  else
  {
    Serial.println("Could not connect to Prop Shield!");
    Serial.print("0x"); Serial.println(c, HEX); 
    Serial.print("0x"); Serial.println(d, HEX); 
    Serial.print("0x"); Serial.println(e, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
}

void loop()
{  
  // One can use the interrupt pins to detect a data ready condition; here we just check the STATUS register for a data ready bit
  if(readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_DR_STATUS) & 0x08)  // When this bit set, all accel axes have new data
  {
    readAccelData(accelCount);       // Read the x/y/z adc values
    ax = (float)accelCount[0]*aRes;  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes;  // also subtract averaged accelerometer biases
    az = (float)accelCount[2]*aRes;  
  }
  
  if(readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_DR_STATUS) & 0x08)  // When this bit set, all mag axes have new data
  {
    readMagData(magCount);         // Read the x/y/z adc values
    mx = (float)magCount[0]*mRes;  // get actual milliGauss value 
    my = (float)magCount[1]*mRes;   
    mz = (float)magCount[2]*mRes;  
  }
  
    if(readByte(FXAS21000_ADDRESS, FXAS21000_DR_STATUS) & 0x08)  // When this bit set, all axes have new data
  {
    readGyroData(gyroCount);  // Read the x/y/z adc values
   // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*gRes - gBias[0];  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*gRes - gBias[1];  
    gz = (float)gyroCount[2]*gRes - gBias[2];  
  } 
          
    Now = micros();
    deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;

    sum += deltat; // sum for averaging filter update rate
    sumCount++;
  
    // Sensors x (y)-axis of the accelerometer/magnetometer is aligned with the x (y)-axis of the gyro on the prop shield;
    // All three sensors have the positive z-axis up.
    // The long axis of the prop shield is in thex-axis direction, which we will designate as North
    // Then x is North, -y is East, and -z is Down for a NED convention
    // Pass gyro rate as rad/s
    MadgwickQuaternionUpdate(-ax, ay, az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f,  mx,  -my, -mz);
//  MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);


   uint32_t delt_t = millis() - count;
   if (delt_t > 500) { // update LCD once per half-second independent of read rate

    // Print out accelerometer data in mgs
    Serial.print("x-acceleration = "); Serial.print(1000.*ax); Serial.println(" mg");   
    Serial.print("y-acceleration = "); Serial.print(1000.*ay); Serial.println(" mg");   
    Serial.print("z-acceleration = "); Serial.print(1000.*az); Serial.println(" mg");
    acceltempCount = readAccelTempData();  // Read the x/y/z adc values
    acceltemperature = (float) acceltempCount * 0.96 + 4.; // Temperature in degrees Centigrade
    Serial.print("Accel Temperature is "); Serial.print(acceltemperature, 1); Serial.println(" C");


    // Print out magnetometer data in mG
    Serial.print("x-magnetic field = "); Serial.print(mx); Serial.println(" mG");   
    Serial.print("y-magnetic field = "); Serial.print(my); Serial.println(" mG");   
    Serial.print("z-magnetic field = "); Serial.print(mz); Serial.println(" mG");  

    Serial.print("x-rate = "); Serial.print(gx); Serial.println(" deg/s");   
    Serial.print("y-rate = "); Serial.print(gy); Serial.println(" deg/s");   
    Serial.print("z-rate = "); Serial.print(gz); Serial.println(" deg/s");  
    gyrotempCount = readGyroTempData();  // Read the x/y/z adc values
    gyrotemperature = (float) gyrotempCount; // Temperature in degrees Centigrade
    Serial.print("Gyro Temperature is "); Serial.print(gyrotemperature, 1); Serial.println(" C");

    MPL3115A2ActiveAltimeterMode(); 
    MPL3115A2readAltitude();  // Read the altitude

    MPL3115A2ActiveBarometerMode(); 
    MPL3115A2readPressure();  // Read the pressure
    
    const int station_elevation_m = 1050.0*0.3048; // Accurate for the roof on my house; convert from feet to meters

    float baroin = pressure/100; // pressure is now in millibars

    // Formula to correct absolute pressure in millbars to "altimeter pressure" in inches of mercury 
    // comparable to weather report pressure
    float part1 = baroin - 0.3; //Part 1 of formula
    const float part2 = 0.0000842288;
    float part3 = pow(part1, 0.190284);
    float part4 = (float)station_elevation_m / part3;
    float part5 = (1.0 + (part2 * part4));
    float part6 = pow(part5, 5.2553026);
    float altimeter_setting_pressure_mb = part1 * part6; // Output is now in adjusted millibars
    baroin = altimeter_setting_pressure_mb * 0.02953;

    
    Serial.print("pressure is "); Serial.print(pressure, 2); Serial.print(" Pa, ");  // Print altitude in meters
    Serial.print("altitude is "); Serial.print(altitude, 2);  Serial.print(" m, "); // Print altitude in meters   
    Serial.print("temperature is "); Serial.print(temperature, 2);  Serial.println(" C"); // Print temperature in C

  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth. 
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    yaw   = atan2f(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asinf(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
    roll  *= 180.0f / PI;
     
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(yaw, 2);
    Serial.print(", ");
    Serial.print(pitch, 2);
    Serial.print(", ");
    Serial.println(roll, 2);
    
    Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz");
    
    digitalWrite(ledPin, !digitalRead(ledPin));
    count = millis(); 
    sumCount = 0;
    sum = 0;    
    }

  // detect all kinds of motion effects
  if(clickMode) {
    
  // One can use the interrupt pins to detect a motion/tap condition; 
  // here we just check the INT_SOURCE register to interpret the motion interrupt condition
    byte Asource = readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_INT_SOURCE);  // Read the interrupt source register
  // Manage sleep/wake interrupts
  if(Asource & 0x80) {  // Check if interrupt source is sleep/wake interrupt
   if(!sleepMode) {
    Serial.println("entering sleep mode");
    sleepMode = true;
    }
    else {
    Serial.println("exiting sleep mode");
    sleepMode = false;
    }
    
    readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_SYSMOD); // Clear sleep interrupt
    delay(1000);                          // Delay a while so we can see the message
  }

  // If interrupt is due to motion control trigger...
  if (Asource & 0x10)  {       // If the p/l bit is set, go check those registers
      portraitLandscapeHandler(); }
    else if (Asource & 0x08)  {// If tap register is set go check that
      tapHandler(); }
    else if (Asource & 0x04) { // If motion detection is set go check that
      motionDetect(); }
 //   else if (source & 0x02)  // If acceleration vector magnitude interrupt
 //     display.setCursor(0,40); display.print("Amagnit!"); 
//      display.display();     // Write message to display


  // One can use the interrupt pins to detect a motion/tap condition; 
  // here we just check the RT_SOURCE register to interpret the rate interrupt condition
  byte Gsource = readByte(FXAS21000_ADDRESS, FXAS21000_RT_SRC);  // Read the interrupt source register
  if(Gsource & 0x40) {  // Check if event flag has been set
 
   // Check source of event
   if(Gsource & 0x20) {    // Z-axis rate event
    if(Gsource & 0x10) {
    Serial.println("-z-axis RT exceeded");
    }
    else {
    Serial.println("+z-axis rate exceeded");
    }
   }
    
 
   // Check source of event
   if(Gsource & 0x08) {    // Y-axis rate event
    if(Gsource & 0x04) {
    Serial.println("-y-axis RT exceeded");
    }
    else {
    Serial.println("+y-axis rate exceeded");
     }
   }
   
      // Check source of event
   if(Gsource & 0x02) {    // X-axis rate event
    if(Gsource & 0x01) {
    Serial.println("-x-axis RT exceeded");
    }
    else {
    Serial.println("+x-axis rate exceeded");
    }
   }
    
  }

}

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////Useful functions
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void getAres() {
  switch (accelFSR)
  {
 	// Possible accelerometer scales (and their register bit settings) are:
	// 2 gs (00), 4 gs (01), 8 gs (10). 
    case AFS_2g:
          aRes = 2.0/8192.0;
          break;
    case AFS_4g:
          aRes = 4.0/8192.0;
          break;
    case AFS_8g:
          aRes = 8.0/8192.0;
          break;
  }
}

void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(FXOS8700CQ_ADDRESS, FXOS8700CQ_OUT_X_MSB, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = (int16_t) (((rawData[0] << 8) | rawData[1])) >> 2; // 14-bit signed integer
  destination[1] = (int16_t) (((rawData[2] << 8) | rawData[3])) >> 2;
  destination[2] = (int16_t) (((rawData[4] << 8) | rawData[5])) >> 2;
}

void readMagData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OUT_X_MSB, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = (int16_t) (((rawData[0] << 8) | rawData[1])); // 16-bit signed integer
  destination[1] = (int16_t) (((rawData[2] << 8) | rawData[3]));
  destination[2] = (int16_t) (((rawData[4] << 8) | rawData[5]));
}

int8_t readAccelTempData()
{
  return (int8_t) readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_TEMP);  // Read the 8-bit 2's complement data register 
}
 
// This function will read the status of the tap source register.
// Print if there's been a single or double tap, and on what axis.
void tapHandler()
{
  byte source = readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_PULSE_SRC);  // Reads the PULSE_SRC register
  if (source & 0x10)  // If AxX bit is set
  {
    if (source & 0x08)  // If DPE (double puls) bit is set
      Serial.print("    Double Tap (2) on X");  // tabbing here for visibility
    else
      Serial.print("Single (1) tap on X");

    if (source & 0x01)  { // If PoIX is set
      Serial.println(" -");
       }
    else {
      Serial.println(" +");
         }
  }
  if (source & 0x20)  // If AxY bit is set
  {
    if (source & 0x08)  // If DPE (double pulse) bit is set
      Serial.print("    Double Tap (2) on Y");
    else
      Serial.print("Single (1) tap on Y");

    if (source & 0x02) { // If PoIY is set
      Serial.println(" -");
       }
    else {
      Serial.println(" +");
      }
}
  if (source & 0x40)  // If AxZ bit is set
  {
    if (source & 0x08)  // If DPE (double puls) bit is set
      Serial.print("    Double Tap (2) on Z");
    else
      Serial.print("Single (1) tap on Z");
    if (source & 0x04) { // If PoIZ is set
      Serial.println(" -"); 
      }
    else {
      Serial.println(" +");
       }
   }
    delay(1000); // Delay a while so we can see the message
}

// This function will read the p/l source register and
// print what direction the sensor is now facing
void portraitLandscapeHandler()
{
  byte pl = readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_PL_STATUS);  // Reads the PL_STATUS register
  switch((pl & 0x06)>>1)  // Check on the LAPO[1:0] bits
  {
  case 0:
    Serial.print("Portrait Up, ");
    delay(1000);
    break;
  case 1:
    Serial.print("Portrait Down, ");
    delay(1000);
    break;
  case 2:
    Serial.print("Landscape Right, ");
    delay(1000);
    break;
  case 3: 
    Serial.print("Landscape Left, ");
    delay(1000);
    break;
  }
  
  if (pl & 0x01) { // Check the BAFRO bit
    Serial.print("Back");
  } else {
    Serial.print("Front");
  }
  if (pl & 0x40) { // Check the LO bit
    Serial.println(", Z-tilt!");

  }
}

// This function will read the motion detection source register and
// print motion direction
void motionDetect()
{
    byte source = readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_FFMT_SRC);
  if((source >> 7) == 1) {  // If Event Active flag set in the FF_MT_SRC register

   if (source & 0x02)  // If XHE bit is set, x-motion detected
  {
    if (source & 0x01)  { // If XHP is 1, x event was negative g
      Serial.println(" -");
       }
    else {
      Serial.println(" +");
       }
  }
  if ((source & 0x08)==0x08)  // If YHE bit is set, y-motion detected
  {
    if (source & 0x04) { // If YHP is set, y event was negative g
      Serial.println(" -");
       }
    else {
      Serial.println(" +");
       }
  }
  if (source & 0x20)  // If ZHE bit is set, z-motion detected
  {
    if (source & 0x10) { // If ZHP is set
      Serial.println(" -"); 
       }
    else {
      Serial.println(" +");
       }
  }
  delay(1000); // Wait a while so we can the message
}
} 

void calibrateFXOS8700CQ()
{
  int32_t accel_bias[3] = {0, 0, 0};
  uint16_t ii, fcount;
  int16_t temp[3];

  Serial.println("Hold sensor flat and motionless for accel calibration!");
  
  // Clear all interrupts by reading the data output and F_STATUS registers
  readAccelData(temp);
  readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_STATUS);
  
  FXOS8700CQStandby();  // Must be in standby to change registers

  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG1, 0x03 << 3); // select 100 Hz ODR
  fcount = 400;                                        // sample for 4 seconds
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_XYZ_DATA_CFG, 0x00);   // select 2 g full scale
  uint16_t accelsensitivity = 4096;                    // 4096 LSB/g

  FXOS8700CQActive();  // Set to active to start collecting data
   
  uint8_t rawData[6];  // x/y/z FIFO accel data stored here
  for(ii = 0; ii < fcount; ii++)   // construct count sums for each axis
  {
  readBytes(FXOS8700CQ_ADDRESS, FXOS8700CQ_OUT_X_MSB, 6, &rawData[0]);  // Read the FIFO data registers into data array
  temp[0] = (int16_t) (((rawData[0] << 8) | rawData[1])) >> 2;
  temp[1] = (int16_t) (((rawData[2] << 8) | rawData[3])) >> 2;
  temp[2] = (int16_t) (((rawData[4] << 8) | rawData[5])) >> 2;
  
  accel_bias[0] += (int32_t) temp[0];
  accel_bias[1] += (int32_t) temp[1];
  accel_bias[2] += (int32_t) temp[2];
  
  delay(10);  // wait for a new data reading (100 Hz)
  }
 
  accel_bias[0] /= (int32_t) fcount; // get average values
  accel_bias[1] /= (int32_t) fcount;
  accel_bias[2] /= (int32_t) fcount;
  
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}

  rawData[0] = (-accel_bias[0]/8) & 0xFF; // get average values
  rawData[1] = (-accel_bias[1]/8) & 0xFF; // get average values
  rawData[2] = (-accel_bias[2]/8) & 0xFF; // get average values
  
  FXOS8700CQStandby();  // Must be in standby to change registers
  
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_OFF_X, rawData[0]); // X-axis compensation  
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_OFF_Y, rawData[1]); // Y-axis compensation  
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_OFF_Z, rawData[2]); // z-axis compensation 

  FXOS8700CQActive();  // Set to active to start reading

    Serial.println("Accel calibration done!");

}
  
  
  // Set up sensor software reset
void FXOS8700CQMagOffset() 
{
  uint16_t ii = 0, sample_count = 0;
  int16_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = {0x8000, 0x8000, 0x8000}, mag_min[3] = {0x7FFF, 0x7FFF, 0x7FFF}, mag_temp[3] = {0, 0, 0};
  float dest1[3] = {0, 0, 0}, dest2[3] = {0, 0, 0};

  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  delay(4000);

  FXOS8700CQStandby();  // Must be in standby to change registers
   
  // Clear all interrupts by reading the data output and F_STATUS registers
  readMagData(mag_temp);
  readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_STATUS);
  // Configure the magnetometer
//  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_CTRL_REG1, 0x80 | magOSR << 2 | 0x03); // Enable auto-calibration, set oversampling, enable hybrid mode 
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_CTRL_REG1, magOSR << 2 | 0x03); // Disable auto-calibration, set oversampling, enable hybrid mode 
 
  FXOS8700CQActive();  // Set to active to start collecting data
   
   sample_count = 512;
   for(ii = 0; ii < sample_count; ii++) {
    readMagData(mag_temp);  // Read the mag data   
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    delay(10);  // at 100 Hz ODR, new mag data is available every 10 ms
    }

    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
    
    dest1[0] = (float) mag_bias[0]*mRes;  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*mRes;   
    dest1[2] = (float) mag_bias[2]*mRes;  
       
    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);
    
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OFF_X_MSB, ((mag_bias[0] << 1) & 0xFF00) >> 8); 
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OFF_X_LSB, ((mag_bias[0] << 1) & 0x00FF)); 
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OFF_Y_MSB, ((mag_bias[1] << 1) & 0xFF00) >> 8); 
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OFF_Y_LSB, ((mag_bias[1] << 1) & 0x00FF)); 
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OFF_Z_MSB, ((mag_bias[2] << 1) & 0xFF00) >> 8); 
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OFF_Z_LSB, ((mag_bias[2] << 1) & 0x00FF)); 
  Serial.println("Mag Calibration done!");

  FXOS8700CQActive();  // Set to active to start reading
}


  void sleepModeFXOS8700CQ()
{
    FXOS8700CQStandby();  // Must be in standby to change registers

  // These settings have to do with setting up the sleep mode
  // Set Auto-WAKE sample frequency when the device is in sleep mode
     writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_ASLP_COUNT, 0x40 ); // sleep after ~36 seconds of inactivity at 6.25 Hz ODR

     writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG1, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG1) & ~(0xC0)); // clear bits 7 and 8
     writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG1, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG1) |  (0xC0)); // select 1.56 Hz sleep mode sample frequency for low power

  // set sleep power mode scheme
     writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG2, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG2) & ~(0x18)); // clear bits 3 and 4
     writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG2, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG2) |  (0x18)); // select low power mode
     
  // Enable auto SLEEP
     writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG2, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG2) & ~(0x04)); // clear bit 2
     writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG2, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG2) |  (0x04)); // enable auto sleep mode

  // set sleep mode interrupt scheme
     writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG3, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG3) & ~(0x7C)); // clear bits 2, 3, 4, 5, and 6
     // select wake on transient, orientation change, pulse, freefall/motion detect, or acceleration vector magnitude
     writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG3, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG3) |  (0x7C)); 
     
   // Enable Auto-SLEEP/WAKE interrupt
     writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG4, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG4) & ~(0x80)); // clear bit 7
     writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG4, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG4) |  (0x80)); // select  Auto-SLEEP/WAKE interrupt enable
 
     FXOS8700CQActive();  // Set to active to start reading
}
  
  
// Set up sensor software reset
void FXOS8700CQReset() 
{
writeByte(FXOS8700CQ_ADDRESS,   FXOS8700CQ_CTRL_REG2, 0x40); // set reset bit to 1 to assert accel software reset to zero at end of boot process
writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_CTRL_REG2, 0x40); // set reset bit to 1 to assert mag software reset to zero at end of boot process
}


// Initialize the FXOS8700CQ registers 
// See the many application notes for more info on setting all of these registers:
// http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=FXOS8700CQQ
// Feel free to modify any values, these are settings that work well for me.
void initFXOS8700CQ()
{
    FXOS8700CQStandby();  // Must be in standby to change registers

    // Configure the accelerometer
    writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_XYZ_DATA_CFG, accelFSR);  // Choose the full scale range to 2, 4, or 8 g.
    writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG1, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG1) & ~(0x38)); // Clear the 3 data rate bits 5:3
    if (accelODR <= 7) writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG1, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG1) | (accelODR << 3));      
    writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG2, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG2) & ~(0x03)); // clear bits 0 and 1
    writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG2, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG2) |  (0x02)); // select normal(00) or high resolution (10) mode
    
    // Configure the magnetometer
//    writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_CTRL_REG1, 0x80 | magOSR << 2 | 0x03); // Enable auto-calibration, set oversampling, enable hybrid mode 
    writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_CTRL_REG1, magOSR << 2 | 0x03); // Disable auto-calibration, set oversampling, enable hybrid mode 
    
    // Configure interrupts 1 and 2
    writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG3, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG3) & ~(0x02)); // clear bits 0, 1 
    writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG3, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG3) |  (0x02)); // select ACTIVE HIGH, push-pull interrupts    
    writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG4, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG4) & ~(0x1D)); // clear bits 0, 3, and 4
    writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG4, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG4) |  (0x1D)); // DRDY, Freefall/Motion, P/L and tap ints enabled  
    writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG5, 0x01);  // DRDY on INT1, P/L and taps on INT2
   
    FXOS8700CQActive();  // Set to active to start reading
}


void accelMotionIntFXOS8700CQ()
{
    FXOS8700CQStandby();  // Must be in standby to change registers
  // This is adapted from Jim Lindblom's sketch originally found on http://www.sparkfun.com
  // Set up portrait/landscape registers - 4 steps:
  // 1. Enable P/L
  // 2. Set the back/front angle trigger points (z-lock)
  // 3. Set the threshold/hysteresis angle
  // 4. Set the debouce rate
  // For more info check out this app note: http://cache.freescale.com/files/sensors/doc/app_note/AN4068.pdf
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_PL_CFG, 0x40);         // 1. Enable P/L
//  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_PL_BF_ZCOMP, 0x04);    // 2. 29deg z-lock 
//  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_P_L_THS_REG, 0x84);    // 3. 45deg thresh, 14deg hyst 
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_PL_COUNT, 0x50);       // 4. debounce counter at 100ms (at 800 hz)

  /* Set up single and double tap - 5 steps:
   1. Set up single and/or double tap detection on each axis individually.
   2. Set the threshold - minimum required acceleration to cause a tap.
   3. Set the time limit - the maximum time that a tap can be above the threshold
   4. Set the pulse latency - the minimum required time between one pulse and the next
   5. Set the second pulse window - maximum allowed time between end of latency and start of second pulse
   for more info check out this app note: http://cache.freescale.com/files/sensors/doc/app_note/AN4072.pdf */
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_PULSE_CFG, 0x7F);  // 1. enable single/double taps on all axes
  // writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_PULSE_CFS, 0x55);  // 1. single taps only on all axes
  // writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_PULSE_CFS, 0x6A);  // 1. double taps only on all axes
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_PULSE_THSX, 0x04);  // 2. x thresh at 0.25g, multiply the value by 0.0625g/LSB to get the threshold
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_PULSE_THSY, 0x04);  // 2. y thresh at 0.25g, multiply the value by 0.0625g/LSB to get the threshold
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_PULSE_THSZ, 0x04);  // 2. z thresh at 0.25g, multiply the value by 0.0625g/LSB to get the threshold
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_PULSE_TMLT, 0x30);  // 3. 2.55s time limit at 100Hz odr, this is very dependent on data rate, see the app note
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_PULSE_LTCY, 0xA0);  // 4. 5.1s 100Hz odr between taps min, this also depends on the data rate
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_PULSE_WIND, 0xFF);  // 5. 10.2s (max value)  at 100 Hz between taps max

  // Set up motion detection
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_FFMT_CFG, 0x58); // Set free fall OR motion flag on x and y axes
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_FFMT_THS, 0x84); // Clear debounce counter when condition no longer obtains, set x/y/z/common threshold to 0.25 g
//  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_FFMT_THS_X_MSB, 0x80); // Set x-axis threshold individually to 0.25 g
//  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_FFMT_THS_X_LSB, 0x04);  
//  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_FFMT_THS_Y_MSB, 0x80); // Set y-axis threshold individually to 0.25 g
//  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_FFMT_THS_Y_LSB, 0x04);  
//  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_FFMT_THS_Z_MSB, 0x80); // Set z-axis threshold individually to 1.25 g
//  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_FFMT_THS_Z_LSB, 0x14);  
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_FFMT_COUNT, 0x08); // Set debounce to 0.08 s at 100 Hz

  // Use user-specified reference value before and after trigger event, enable accelerometer magnitude function  
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_VECM_CFG, 0x78); 
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_VECM_THS_MSB, 0x80 | 0x0F); // Clear debounce counter when event falls below threshold; bit 4:0 MSB THS 
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_VECM_THS_LSB, 0xFF);        // Specify threshold at 0.3 g = 1228 * 0.244 g at FSR of +/-2 g
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_VECM_CNT, 0x01);            // Set debounce time to 80 x 1.25 ms (in high res mode) = 100 ms at 400 Hz (200 Hz hybrid mode)
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_VECM_INITX_MSB, 0x00);      // set x-axis vector magnitude reference at 0 g
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_VECM_INITX_LSB, 0x00);
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_VECM_INITY_MSB, 0x00);      // set y-axis vector magnitude reference at 0 g
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_VECM_INITY_LSB, 0x00);
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_VECM_INITZ_MSB, 0x10);      // set z-axis vector magnitude reference at +1 g = 4096 at +/- 2 g FSR
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_VECM_INITZ_LSB, 0x00);
  
  FXOS8700CQActive();  // Set to active to start reading
}


// Sets the FXOS8700CQ to standby mode.
// It must be in standby to change most register settings
void FXOS8700CQStandby()
{
  byte c = readByte(FXOS8700CQ_ADDRESS, 0x2A);
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG1, c & ~(0x01));
}


// Sets the FXOS8700CQ to active mode.
// Needs to be in this mode to output data
void FXOS8700CQActive()
{
  byte c = readByte(FXOS8700CQ_ADDRESS, 0x2A);
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG1, c | 0x01);
}

void getGres() {
  switch (gyroFSR)
  {
   // Possible gyro scales (and their register bit settings) are:
  // 200 DPS (11), 400 DPS (10), 800 DPS (01), and 1600 DPS  (00). 
    case GFS_1600DPS:
          gRes = 1600.0/8192.0;
          break;
    case GFS_800DPS:
          gRes = 800.0/8192.0;
          break;
    case GFS_400DPS:
          gRes = 400.0/8192.0;
          break;           
    case GFS_200DPS:
          gRes = 200.0/8192.0;
  }
}

void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(FXAS21000_ADDRESS, FXAS21000_OUT_X_MSB, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = (int16_t) (((rawData[0] << 8) | rawData[1])) >> 2; // signer 14-bit integers
  destination[1] = (int16_t) (((rawData[2] << 8) | rawData[3])) >> 2;
  destination[2] = (int16_t) (((rawData[4] << 8) | rawData[5])) >> 2;
}

int8_t readGyroTempData()
{
  return (int8_t) readByte(FXAS21000_ADDRESS, FXAS21000_TEMP);  // Read the 8-bit 2's complement data register 
}

 
void calibrateFXAS21000(float * gBias)
{
  int32_t gyro_bias[3] = {0, 0, 0};
  uint16_t ii, fcount;
  int16_t temp[3];

    Serial.println("Hold sensor flat and motionless for gyro calibration!");
  
  // Clear all interrupts by reading the data output and STATUS registers
  readGyroData(temp);
  readByte(FXAS21000_ADDRESS, FXAS21000_STATUS);
  
  FXAS21000Standby();  // Must be in standby to change registers

  writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1, 0x08);   // select 50 Hz ODR
  fcount = 400;                                     // sample for 4 second
  writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG0, 0x03);   // select 200 deg/s full scale
  uint16_t gyrosensitivity = 41;                   // 40.96 LSB/deg/s

  FXAS21000Active();  // Set to active to start collecting data
   
  uint8_t rawData[6];  // x/y/z FIFO accel data stored here
  for(ii = 0; ii < fcount; ii++)   // construct count sums for each axis
  {
  readBytes(FXAS21000_ADDRESS, FXAS21000_OUT_X_MSB, 6, &rawData[0]);  // Read the FIFO data registers into data array
  temp[0] = (int16_t) (((rawData[0] << 8) | rawData[1])) >> 2;
  temp[1] = (int16_t) (((rawData[2] << 8) | rawData[3])) >> 2;
  temp[2] = (int16_t) (((rawData[4] << 8) | rawData[5])) >> 2;
  
  gyro_bias[0] += (int32_t) temp[0];
  gyro_bias[1] += (int32_t) temp[1];
  gyro_bias[2] += (int32_t) temp[2];
  
  delay(25); // wait for next data sample at 50 Hz rate
  }
 
  gyro_bias[0] /= (int32_t) fcount; // get average values
  gyro_bias[1] /= (int32_t) fcount;
  gyro_bias[2] /= (int32_t) fcount;
  
  gBias[0] = (float)gyro_bias[0]/(float) gyrosensitivity; // get average values
  gBias[1] = (float)gyro_bias[1]/(float) gyrosensitivity; // get average values
  gBias[2] = (float)gyro_bias[2]/(float) gyrosensitivity; // get average values

  Serial.println("Gyro calibration done!");

  FXAS21000Ready();  // Set to ready
}
  
  
// Set up sensor software reset
void FXAS21000Reset() 
{
writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1, 0x20); // set reset bit to 1 to assert software reset to zero at end of boot process
delay(100);
while(!(readByte(FXAS21000_ADDRESS, FXAS21000_INT_SRC_FLAG) & 0x08))  { // wait for boot end flag to be set
}
}



// Initialize the FXAS21000 registers 
// See the many application notes for more info on setting all of these registers:
// http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=FXAS21000Q
// Feel free to modify any values, these are settings that work well for me.
void initFXAS21000()
{
  FXAS21000Standby();  // Must be in standby to change registers

  // Set up the full scale range to 200, 400, 800, or 1600 deg/s.
  writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG0, gyroFSR);      // write FSR

  // Setup the 3 data rate bits, 4:2
  if(gyroODR < 8) { // data rate can only be 0 to 7
  writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1, gyroODR << 2); 
  }

  // Disable FIFO, route FIFO and rate threshold interrupts to INT2, enable data ready interrupt, route to INT1
  // Active HIGH, push-pull output driver on interrupts
  writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG2,  0x0E);
  
  // Set up rate threshold detection; at max rate threshold = FSR; rate threshold = THS*FSR/128
  writeByte(FXAS21000_ADDRESS, FXAS21000_RT_CFG, 0x07);         // enable rate threshold detection on all axes
  writeByte(FXAS21000_ADDRESS, FXAS21000_RT_THS, 0x00 | 0x0D);  // unsigned 7-bit THS, set to one-tenth FSR; set clearing debounce counter
  writeByte(FXAS21000_ADDRESS, FXAS21000_RT_COUNT, 0x04);       // set to 4 (can set up to 255)
        
  FXAS21000Active();  // Set to active to start reading
}


// Sets the FXAS21000 to standby mode.
// It must be in standby to change most register settings
void FXAS21000Standby()
{
  byte c = readByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1);
  writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1, c & ~(0x03));  // Clear bits 0 and 1; standby mode
}

// Sets the FXAS21000 to active mode.
// Needs to be in this mode to output data
void FXAS21000Ready()
{
  byte c = readByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1);
  writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1, c & ~(0x03));  // Clear bits 0 and 1; standby mode
  writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1, c |   0x01);   // Set bit 0 to 1, ready mode; no data acquisition yet
}

// Sets the FXAS21000 to active mode.
// Needs to be in this mode to output data
void FXAS21000Active()
{
  byte c = readByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1);
 writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1, c & ~(0x03));  // Clear bits 0 and 1; standby mode
 writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1, c |   0x02);   // Set bit 1 to 1, active mode; data acquisition enabled
}


void MPL3115A2readAltitude() // Get altitude in meters and temperature in centigrade
{
  uint8_t rawData[5];  // msb/csb/lsb pressure and msb/lsb temperature stored in five contiguous registers 

// We can read the data either by polling or interrupt; see data sheet for relative advantages
// First we try hardware interrupt, which should take less power, etc.
// while (digitalRead(int1Pin) == LOW); // Wait for interrupt pin int1Pin to go HIGH
// digitalWrite(int1Pin, LOW);  // Reset interrupt pin int1Pin
 while((readByte(MPL3115A2_ADDRESS, MPL3115A2_INT_SOURCE) & 0x80) == 0); // Check that the interrupt source is a data ready interrupt
// or use a polling method
// Check data read status; if PTDR (bit 4) not set, then
// toggle OST bit to cause sensor to immediately take a reading
// Setting the one shot toggle is the way to get faster than 1 Hz data read rates
// while ((readByte(MPL3115A2_ADDRESS, MPL3115A2_STATUS) & 0x08) == 0);  MPL3115A2toggleOneShot(); 
  
  readBytes(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_MSB, 5, &rawData[0]);  // Read the five raw data registers into data array

// Altitude bytes-whole altitude contained defined by msb, csb, and first two bits of lsb, fraction by next two bits of lsb
  uint8_t msbA = rawData[0];
  uint8_t csbA = rawData[1];
  uint8_t lsbA = rawData[2];
// Temperature bytes
  uint8_t msbT = rawData[3];
  uint8_t lsbT = rawData[4];

 // Calculate altitude 
  int16_t altitude_whole = ((int16_t)msbA << 8 | (int16_t)csbA ) ; // Construct signed 16-bit whole number altitude
 
  lsbA &= 0xF0; // Keep only bits 5 - 7, the fractional altitude
  lsbA >>= 4; // Shift to get the fractional altitude
  float altitude_frac = (float) lsbA/16.0; // Convert to fractional altitude in meters

  altitude = (float) (altitude_whole) + altitude_frac; // Combine whole and fractional parts to get entire pressure in Pascal

 // Calculate temperature 
  int16_t temperature_whole = ((int16_t)msbT << 8 | lsbT ) ; // Construct signed 16-bit whole number temperature
  temperature_whole >>= 8;
 
  lsbT &= 0xF0; // Keep only bits 5 - 7, the fractional temperature
  lsbT >>= 4; // Shift to get the fractional temperature
  float temperature_frac = (float) lsbT/16.0; // Convert to fractional temperature in Centigrade

  temperature = (float) (temperature_whole) + temperature_frac; // Combine whole and fractional parts to get entire temperature in Centigrade
}

void MPL3115A2readPressure()
{
  uint8_t  rawData[5];  // msb/csb/lsb pressure and msb/lsb temperature stored in five contiguous registers

// We can read the data either by polling or interrupt; see data sheet for relative advantages
// First we try hardware interrupt, which should take less power, etc.
// while (digitalRead(int1Pin) == LOW); // Wait for interrupt pin int1Pin to go HIGH
// digitalWrite(int1Pin, LOW);  // Reset interrupt pin int1Pin
 while((readByte(MPL3115A2_ADDRESS, MPL3115A2_INT_SOURCE) & 0x80) == 0); // Check that the interrupt source is a data ready interrupt
// or use a polling method
// Check data read status; if PTDR (bit 4) not set, then
// toggle OST bit to cause sensor to immediately take a reading
// Setting the one shot toggle is the way to get faster than 1 Hz data read rates
 //while ((readByte(MPL3115A2_ADDRESS, MPL3115A2_STATUS) & 0x08) == 0);  MPL3115A2toggleOneShot(); 
 
  readBytes(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_MSB, 5, &rawData[0]);  // Read the five raw data registers into data array

// Pressure bytes
  uint8_t msbP = rawData[0];
  uint8_t csbP = rawData[1];
  uint8_t lsbP = rawData[2];
// Temperature bytes
  uint8_t msbT = rawData[3];
  uint8_t lsbT = rawData[4]; 
 
// Calculate pressure 
   int32_t pressure_whole =   ((int32_t)msbP << 16 |  (int32_t)csbP << 8 |  (int32_t)lsbP) ; // Construct whole number pressure
  pressure_whole >>= 6; // Only two most significant bits of lsbP contribute to whole pressure; its an 18-bit number
 
  lsbP &= 0x30; // Keep only bits 5 and 6, the fractional pressure
  lsbP >>= 4; // Shift to get the fractional pressure in terms of quarters of a Pascal
  float pressure_frac = (float) lsbP/4.0; // Convert numbers of fractional quarters to fractional pressure n Pasacl

  pressure = (float) (pressure_whole) + pressure_frac; // Combine whole and fractional parts to get entire pressure in Pascal

 // Calculate temperature 
  int16_t temperature_whole = ((int16_t)msbT << 8 | lsbT ) ; // Construct signed 16-bit whole number temperature
  temperature_whole >>= 8;

  lsbT &= 0xF0; // Keep only bits 5 - 7, the fractional temperature
  lsbT >>= 4; // Shift to get the fractional temperature
  float temperature_frac = (float) lsbT/16.0; // Convert to fractional temperature in Centigrade

  temperature = (float) (temperature_whole) + temperature_frac; // Combine whole and fractional parts to get entire temperature in Centigrade
}

/*
=====================================================================================================
Define functions according to 
"Data Manipulation and Basic Settings of the MPL3115A2 Command Line Interface Drive Code"
by Miguel Salhuana
Freescale Semiconductor Application Note AN4519 Rev 0.1, 08/2012
=====================================================================================================
*/
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Clears then sets OST bit which causes the sensor to immediately take another reading
void MPL3115A2toggleOneShot()
{
    MPL3115A2Active();  // Set to active to start reading
    uint8_t c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1);
    writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c & ~(1<<1)); // Clear OST (bit 1)
    c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1);
    writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c | (1<<1)); // Set OST bit to 1
}
    
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Set the Outputting Sample Rate
void MPL3115A2SampleRate(uint8_t samplerate)
{
  MPL3115A2Standby();  // Must be in standby to change registers

  uint8_t c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1);
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c & ~(0x38)); // Clear OSR bits 3,4,5
  if(samplerate < 8) { // OSR between 1 and 7
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c | (samplerate << 3));  // Write OSR to bits 3,4,5
  }
  
  MPL3115A2Active();  // Set to active to start reading
 }
 
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Initialize the MPL3115A2 registers for FIFO mode
void initFIFOMPL3115A2()
{
  // Clear all interrupts by reading the data output registers
  uint8_t temp;
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_MSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_CSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_LSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_T_MSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_T_LSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_F_STATUS);
  
   MPL3115A2Standby();  // Must be in standby to change registers
  
  // Set CTRL_REG4 register to configure interupt enable
  // Enable data ready interrupt (bit 7), enable FIFO (bit 6), enable pressure window (bit 5), temperature window (bit 4),
  // pressure threshold (bit 3), temperature threshold (bit 2), pressure change (bit 1) and temperature change (bit 0)
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG4, 0x40);  // enable FIFO
  
  //  Configure INT 1 for data ready, all other (inc. FIFO) interrupts to INT2
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG5, 0x80); 
  
  // Set CTRL_REG3 register to configure interupt signal type
  // Active HIGH, push-pull interupts INT1 and INT 2
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG3, 0x22); 
  
  // Set FIFO mode
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_F_SETUP, 0x00); // Clear FIFO mode
// In overflow mode, when FIFO fills up, no more data is taken until the FIFO registers are read
// In watermark mode, the oldest data is overwritten by new data until the FIFO registers are read
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_F_SETUP, 0x80); // Set F_MODE to interrupt when overflow = 32 reached
//  writeByte(MPL3115A2_ADDRESS, F_SETUP, 0x60); // Set F_MODE to accept 32 data samples and interrupt when watermark = 32 reached

  MPL3115A2Active();  // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Initialize the MPL3115A2 for realtime data collection 
void initRealTimeMPL3115A2()
{
  // Clear all interrupts by reading the data output registers
  uint8_t temp;
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_MSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_CSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_LSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_T_MSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_T_LSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_F_STATUS);
  
   MPL3115A2Standby();  // Must be in standby to change registers
  
  // Set CTRL_REG4 register to configure interupt enable
  // Enable data ready interrupt (bit 7), enable FIFO (bit 6), enable pressure window (bit 5), temperature window (bit 4),
  // pressure threshold (bit 3), temperature threshold (bit 2), pressure change (bit 1) and temperature change (bit 0)
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG4, 0x80);  
  
  //  Configure INT 1 for data ready, all other interrupts to INT2
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG5, 0x80); 
  
  // Set CTRL_REG3 register to configure interupt signal type
  // Active HIGH, push-pull interupts INT1 and INT 2
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG3, 0x22); 
  
  // Set FIFO mode
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_F_SETUP, 0x00); // disable FIFO mode
  
  MPL3115A2Active();  // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Set the Auto Acquisition Time Step
void MPL3115A2TimeStep(uint8_t ST_Value)
{
 MPL3115A2Standby(); // First put device in standby mode to allow write to registers
 
 uint8_t c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG2); // Read contents of register CTRL_REG2
 if (ST_Value <= 0xF) {
 writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG2, (c | ST_Value)); // Set time step n from 0x0 to 0xF (bits 0 - 3) for time intervals from 1 to 32768 (2^n) seconds
 }
 
 MPL3115A2Active(); // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Enable the pressure and temperature event flags
 // Bit 2 is general data ready event mode on new Pressure/Altitude or temperature data
 // Bit 1 is event flag on new Pressure/Altitude data
 // Bit 0 is event flag on new Temperature data
void MPL3115A2enableEventflags()
{
  MPL3115A2Standby();  // Must be in standby to change registers
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_PT_DATA_CFG, 0x07); //Enable all three pressure and temperature event flags
  MPL3115A2Active();  // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Enter Active Altimeter mode
void MPL3115A2ActiveAltimeterMode()
{
 MPL3115A2Standby(); // First put device in standby mode to allow write to registers
 uint8_t c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1); // Read contents of register CTRL_REG1
 writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c | (0x80)); // Set ALT (bit 7) to 1
 MPL3115A2Active(); // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Enter Active Barometer mode
void MPL3115A2ActiveBarometerMode()
{
 MPL3115A2Standby(); // First put device in standby mode to allow write to registers
 uint8_t c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1); // Read contents of register CTRL_REG1
 writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c & ~(0x80)); // Set ALT (bit 7) to 0
 MPL3115A2Active(); // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Software resets the MPL3115A2.
// It must be in standby to change most register settings
void MPL3115A2Reset()
{
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, 0x04); // Set RST (bit 2) to 1
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Sets the MPL3115A2 to standby mode.
// It must be in standby to change most register settings
void MPL3115A2Standby()
{
  uint8_t c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1); // Read contents of register CTRL_REG1
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c & ~(0x01)); // Set SBYB (bit 0) to 0
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Sets the MPL3115A2 to active mode.
// Needs to be in this mode to output data
void MPL3115A2Active()
{
  uint8_t c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1); // Read contents of register CTRL_REG1
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c | 0x01); // Set SBYB (bit 0) to 1
}

// simple function to scan for I2C devices on the bus
void I2Cscan() 
{
  // scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
      Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

// I2C read/write functions 

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data   
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

        void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
//  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  Wire.endTransmission(I2C_NOSTOP);             // Send the Tx buffer, but send a restart to keep connection alive  uint8_t i = 0;
        uint8_t i = 0;
        Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
  while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer

}
