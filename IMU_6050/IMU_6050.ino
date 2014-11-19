#include <Kalman.h>
#include <math.h>
#include <PIDCont.h>
#include <Wire.h>

#include "IMU_6050.h"

// Global definitions
#define ROLL_PID_KP    0.250
#define ROLL_PID_KI    0
#define ROLL_PID_KD    0
#define ROLL_PID_MIN  -52.0
#define ROLL_PID_MAX   52.0

#define PITCH_PID_KP   0.250
#define PITCH_PID_KI   0
#define PITCH_PID_KD   0
#define PITCH_PID_MIN -52.0
#define PITCH_PID_MAX  52.0

#define YAW_PID_KP     0.680
#define YAW_PID_KI     0
#define YAW_PID_KD     0
#define YAW_PID_MIN   -52.0
#define YAW_PID_MAX    52.0

#define ANGLEX_KP      5.0
#define ANGLEX_KI      0.02
#define ANGLEX_KD     -0.015
#define ANGLEX_MIN    -104.0
#define ANGLEX_MAX     104.0

#define ANGLEY_KP      5.0
#define ANGLEY_KI      0.02
#define ANGLEY_KD     -0.015
#define ANGLEY_MIN    -104.0
#define ANGLEY_MAX     104.0

// Global variables
const int ledPin = 13;
int incomingByte;
int temp;
float error_Y[5][2];
int count = 0;
PIDCont PIDroll,PIDpitch,PIDyaw,PIDangleX,PIDangleY;
int setX = 0;
int setY = 0;
int setZ = 0;
float gx_aver=0;
float gy_aver=0;
float gz_aver=0;
float motor[4];  //current motor speed
float min_speed; //used to set speed modes between settings
float max_speed; //prevents random jumps or shut-offs
int throttle = 0;
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
float base_x_accel;
float base_y_accel;
float base_z_accel;
float base_x_gyro;
float base_y_gyro;
float base_z_gyro;
unsigned long last_read_time;
float last_x_angle;  // These are the filtered angles
float last_y_angle;
float last_z_angle;  
float last_gyro_x_angle;  // Store the gyro angles to compare drift
float last_gyro_y_angle;
float last_gyro_z_angle;
float last_x_kalangle;  //Kalman angle
float last_y_kalangle;
float last_z_kalangle;
typedef union accel_t_gyro_union
{
  struct
  {
    uint8_t x_accel_h;
    uint8_t x_accel_l;
    uint8_t y_accel_h;
    uint8_t y_accel_l;
    uint8_t z_accel_h;
    uint8_t z_accel_l;
    uint8_t t_h;
    uint8_t t_l;
    uint8_t x_gyro_h;
    uint8_t x_gyro_l;
    uint8_t y_gyro_h;
    uint8_t y_gyro_l;
    uint8_t z_gyro_h;
    uint8_t z_gyro_l;
  } reg;
  struct 
  {
    int x_accel;
    int y_accel;
    int z_accel;
    int temperature;
    int x_gyro;
    int y_gyro;
    int z_gyro;
  } value;
};

// Global function prototypes
void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro, float kx, float ky, float kz);
int read_gyro_accel_vals(uint8_t* accel_t_gyro_ptr);
void calibrate_sensors();

inline unsigned long get_last_time() {return last_read_time;}
inline float get_last_x_angle() {return last_x_angle;}
inline float get_last_y_angle() {return last_y_angle;}
inline float get_last_z_angle() {return last_z_angle;}
inline float get_last_gyro_x_angle() {return last_gyro_x_angle;}
inline float get_last_gyro_y_angle() {return last_gyro_y_angle;}
inline float get_last_gyro_z_angle() {return last_gyro_z_angle;}
inline float get_last_x_kalangle() {return last_x_kalangle;}
inline float get_last_y_kalangle() {return last_y_kalangle;}
inline float get_last_z_kalangle() {return last_z_kalangle;}

/********
* SETUP *
*********/

void setup()
{      
  int error;
  uint8_t c;
  accel_t_gyro_union accel_t_gyro;
  
  Serial.begin(9600);
  Wire.begin();
  error = MPU6050_read (MPU6050_WHO_AM_I, &c, 1);
  error = MPU6050_read (MPU6050_PWR_MGMT_2, &c, 1);
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);
  calibrate_sensors();  
  set_last_read_angle_data(millis(), 0, 0, 0, 0, 0, 0, 0, 0, 0);
  float accel_x = accel_t_gyro.value.x_accel;
  float accel_y = accel_t_gyro.value.y_accel;
  float accel_z = accel_t_gyro.value.z_accel;
  float RADIANS_TO_DEGREES = 180/3.14159;
  float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
  float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
  float accel_angle_z = 0;
  kalmanX.setAngle(accel_angle_x); // Set starting angle
  kalmanY.setAngle(accel_angle_y);
  pinMode(ledPin, OUTPUT);
  for(temp = 0; temp < 4; temp++)
  {
    motor[temp] = 0;
  }
  min_speed = 0;
  max_speed = 0;
  
  PIDroll.ChangeParameters(ROLL_PID_KP,ROLL_PID_KI,ROLL_PID_KD,ROLL_PID_MIN,ROLL_PID_MAX);
  PIDpitch.ChangeParameters(PITCH_PID_KP,PITCH_PID_KI,PITCH_PID_KD,PITCH_PID_MIN,PITCH_PID_MAX);
  PIDyaw.ChangeParameters(YAW_PID_KP,YAW_PID_KI,YAW_PID_KD,YAW_PID_MIN,YAW_PID_MAX);
  PIDangleX.ChangeParameters(ANGLEX_KP,ANGLEX_KI,ANGLEX_KD,ANGLEX_MIN,ANGLEX_MAX);
  PIDangleY.ChangeParameters(ANGLEY_KP,ANGLEY_KI,ANGLEY_KD,ANGLEY_MIN,ANGLEY_MAX);
  
  PIDangleX.resetI();
  PIDangleY.resetI();
}

/************
* Main loop *
*************/
void loop()
{
  int error;
  double dT;
  accel_t_gyro_union accel_t_gyro;
  
  float kalAngleX, kalAngleY, kalAngleZ; // Calculate the angle using a Kalman filter
  float kalAngleX_last, kalAngleY_last, kalAngleZ_last;
      
  // Read the raw values.
  error = read_gyro_accel_vals((uint8_t*) &accel_t_gyro);
  
  // Get the time of reading for rotation computations
  unsigned long t_now = millis();
   

  // Convert gyro values to degrees/sec
  float FS_SEL = 131;

  float gyro_x = (accel_t_gyro.value.x_gyro - base_x_gyro)/FS_SEL;
  float gyro_y = (accel_t_gyro.value.y_gyro - base_y_gyro)/FS_SEL;
  float gyro_z = (accel_t_gyro.value.z_gyro - base_z_gyro)/FS_SEL;
  
  
  // Get raw acceleration values
  //float G_CONVERT = 16384;
  float accel_x = accel_t_gyro.value.x_accel;
  float accel_y = accel_t_gyro.value.y_accel;
  float accel_z = accel_t_gyro.value.z_accel;
  
  // Get angle values from accelerometer
  float RADIANS_TO_DEGREES = 180/3.14159;
//  float accel_vector_length = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
  float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
  float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;

  float accel_angle_z = 0;
  
  // Compute the (filtered) gyro angles
  float dt =(t_now - get_last_time())/1000.0;
  float gyro_angle_x = gyro_x*dt + get_last_x_angle();
  float gyro_angle_y = gyro_y*dt + get_last_y_angle();
  float gyro_angle_z = gyro_z*dt + get_last_z_angle();
  
  // Compute the drifting gyro angles
  float unfiltered_gyro_angle_x = gyro_x*dt + get_last_gyro_x_angle();
  float unfiltered_gyro_angle_y = gyro_y*dt + get_last_gyro_y_angle();
  float unfiltered_gyro_angle_z = gyro_z*dt + get_last_gyro_z_angle();
  
  // Apply the complementary filter to figure out the change in angle - choice of alpha is
  // estimated now.  Alpha depends on the sampling rate...
  float alpha = 0.96;
  float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
  float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
  float angle_z = gyro_angle_z;  //Accelerometer doesn't give z-angle
  
  
  //KALMAN filter
  kalAngleX = kalmanX.getAngle(accel_angle_x, gyro_x, dt);
  kalAngleY = kalmanY.getAngle(accel_angle_y, gyro_y, dt);
  kalAngleZ = gyro_angle_z;
  
  
  // Update the saved data with the latest values
  set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z, kalAngleX, kalAngleY, kalAngleZ);
    
  if(Serial.available() > 0)
  {
    incomingByte = Serial.read();
    if(incomingByte == 'H')
    {
        digitalWrite(ledPin, HIGH);
    }
    if(incomingByte == 'L')
    {
      digitalWrite(ledPin, LOW);
    }
    if(incomingByte == 'A')
    {
      throttle = 100;
      min_speed = 100;
      max_speed = 100;
    }
    if(incomingByte == 'O')
    {
      throttle = 202;
      min_speed = 150;
      max_speed = 254;
    }
    if(incomingByte == 'F')
    {
      throttle = 0;
      min_speed = 0;
      max_speed = 0;
    }
  }
  
  //prevent motors from powering up for one minute from start up...
  //this gives the gyro some time to start up
  if(millis() < 10000)
  {
    Serial.print(millis());
    Serial.print('\n');
  }
  else
  {
    int PIDroll_val= (int)PIDroll.Compute((float)kalAngleY);
    int PIDpitch_val= (int)PIDpitch.Compute((float)kalAngleX);
    int PIDyaw_val= (int)PIDyaw.Compute((float)kalAngleZ);
//    Serial.print("PIDroll_val: ");
//    Serial.print(PIDroll_val);
//    Serial.print('\n');
//    Serial.print("PIDpitch_val: ");
//    Serial.print(PIDpitch_val);
//    Serial.print('\n');
//    Serial.print("PIDyaw_val: ");
//    Serial.print(kalAngleZ);
//    Serial.print('\n');
    motor[1]=throttle+PIDroll_val+PIDpitch_val+PIDyaw_val; //11
    motor[2]=throttle-PIDroll_val-PIDpitch_val-PIDyaw_val; //9
    motor[0]=throttle+PIDroll_val-PIDpitch_val-PIDyaw_val; // 8
    motor[3]=throttle-PIDroll_val+PIDpitch_val+PIDyaw_val; //10
    
    //prevents motors from firing at incorrect times or powering off in flight
    for(temp = 0; temp < 4; temp++)
    {
      if(motor[temp] > max_speed)
      {
        motor[temp] = max_speed;
      }
      else if(motor[temp] < min_speed)
      {
        motor[temp] = min_speed;
      }
    }
    
    Serial.print("Motor 11: ");
    Serial.print(int(round(motor[3])));
    Serial.print('\n');
    analogWrite(8 , int(round(motor[0])));
//    analogWrite(9 , int(round(motor[2])));
    analogWrite(10, int(round(motor[3])));
//    analogWrite(11, int(round(motor[1])));
  }  
}


// --------------------------------------------------------
// MPU6050_read
//
// This is a common function to read multiple bytes 
// from an I2C device.
//
// It uses the boolean parameter for Wire.endTransMission()
// to be able to hold or release the I2C-bus. 
// This is implemented in Arduino 1.0.1.
//
// Only this function is used to read. 
// There is no function for a single byte.
//
int MPU6050_read(int start, uint8_t *buffer, int size)
{
  int i, n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return (-10);

  n = Wire.endTransmission(false);    // hold the I2C-bus
  if (n != 0)
    return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
  i = 0;
  while(Wire.available() && i<size)
  {
    buffer[i++]=Wire.read();
  }
  if ( i != size)
    return (-11);

  return (0);  // return : no error
}


// --------------------------------------------------------
// MPU6050_write
//
// This is a common function to write multiple bytes to an I2C device.
//
// If only a single register is written,
// use the function MPU_6050_write_reg().
//
// Parameters:
//   start : Start address, use a define for the register
//   pData : A pointer to the data to write.
//   size  : The number of bytes to write.
//
// If only a single register is written, a pointer
// to the data has to be used, and the size is
// a single byte:
//   int data = 0;        // the data to write
//   MPU6050_write (MPU6050_PWR_MGMT_1, &c, 1);
//
int MPU6050_write(int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);        // write the start address
  if (n != 1)
    return (-20);

  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (-21);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);

  return (0);         // return : no error
}

// --------------------------------------------------------
// MPU6050_write_reg
//
// An extra function to write a single register.
// It is just a wrapper around the MPU_6050_write()
// function, and it is only a convenient function
// to make it easier to write a single register.
//
int MPU6050_write_reg(int reg, uint8_t data)
{
  int error;

  error = MPU6050_write(reg, &data, 1);

  return (error);
}

void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro, float kx, float ky, float kz) {
  last_read_time = time;
  last_x_angle = x;
  last_y_angle = y;
  last_z_angle = z;
  last_gyro_x_angle = x_gyro;
  last_gyro_y_angle = y_gyro;
  last_gyro_z_angle = z_gyro;
  last_x_kalangle = kx;
  last_y_kalangle = ky;
  last_z_kalangle = kz;
}

int read_gyro_accel_vals(uint8_t* accel_t_gyro_ptr) {
  // Read the raw values.
  // Read 14 bytes at once, 
  // containing acceleration, temperature and gyro.
  // With the default settings of the MPU-6050,
  // there is no filter enabled, and the values
  // are not very stable.  Returns the error value
  
  accel_t_gyro_union* accel_t_gyro = (accel_t_gyro_union *) accel_t_gyro_ptr;
   
  int error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) accel_t_gyro, sizeof(*accel_t_gyro));

  // Swap all high and low bytes.
  // After this, the registers values are swapped, 
  // so the structure name like x_accel_l does no 
  // longer contain the lower byte.
  uint8_t swap;
  #define SWAP(x,y) swap = x; x = y; y = swap

  SWAP ((*accel_t_gyro).reg.x_accel_h, (*accel_t_gyro).reg.x_accel_l);
  SWAP ((*accel_t_gyro).reg.y_accel_h, (*accel_t_gyro).reg.y_accel_l);
  SWAP ((*accel_t_gyro).reg.z_accel_h, (*accel_t_gyro).reg.z_accel_l);
  SWAP ((*accel_t_gyro).reg.t_h, (*accel_t_gyro).reg.t_l);
  SWAP ((*accel_t_gyro).reg.x_gyro_h, (*accel_t_gyro).reg.x_gyro_l);
  SWAP ((*accel_t_gyro).reg.y_gyro_h, (*accel_t_gyro).reg.y_gyro_l);
  SWAP ((*accel_t_gyro).reg.z_gyro_h, (*accel_t_gyro).reg.z_gyro_l);

  return error;
}

// The sensor should be motionless on a horizontal surface 
//  while calibration is happening
void calibrate_sensors() {
  int                   num_readings = 10;
  float                 x_accel = 0;
  float                 y_accel = 0;
  float                 z_accel = 0;
  float                 x_gyro = 0;
  float                 y_gyro = 0;
  float                 z_gyro = 0;
  accel_t_gyro_union    accel_t_gyro;
  
  // Calibration start
  // Discard the first set of values read from the IMU
  read_gyro_accel_vals((uint8_t *) &accel_t_gyro);
  
  // Read and average the raw values from the IMU
  for (int i = 0; i < num_readings; i++) {
    read_gyro_accel_vals((uint8_t *) &accel_t_gyro);
    x_accel += accel_t_gyro.value.x_accel;
    y_accel += accel_t_gyro.value.y_accel;
    z_accel += accel_t_gyro.value.z_accel;
    x_gyro += accel_t_gyro.value.x_gyro;
    y_gyro += accel_t_gyro.value.y_gyro;
    z_gyro += accel_t_gyro.value.z_gyro;
    delay(100);
  }
  x_accel /= num_readings;
  y_accel /= num_readings;
  z_accel /= num_readings;
  x_gyro /= num_readings;
  y_gyro /= num_readings;
  z_gyro /= num_readings;
  
  // Store the raw calibration values globally
  base_x_accel = x_accel;
  base_y_accel = y_accel;
  base_z_accel = z_accel;
  base_x_gyro = x_gyro;
  base_y_gyro = y_gyro;
  base_z_gyro = z_gyro;
  
  // Calibration finished
}
