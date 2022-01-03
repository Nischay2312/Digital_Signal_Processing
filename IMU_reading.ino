#include "MPU6050.h"
#include <Wire.h>

#define I2C_SDA 33
#define I2C_SCL 32

#define IMU_add 0x68

#define Gyro_sen 131.0
#define Acc_sen 8192.0

//Filter coefficients
#define pi 3.14285714286 
#define fs 200.0
#define coeff  2/(1/fs)
#define frequency 0.85
#define cutoff  frequency*2.0*pi
float a1 = (coeff - cutoff)/(coeff + cutoff);
float b0 = cutoff/(coeff+cutoff);
float b1 = cutoff/(coeff+cutoff);

#define complementary_factor 0.98

//Initialize MPU6050. General initialization
void mpu6050_init()/*int print_extension_type*/
{
  //Address of MPU6050 when AD0 is low is 0x68
  Wire.beginTransmission(IMU_add);
  //Setting the sample rate of gyro to 500Hz.
  //sample_rate = 8kHz/(1+value(decimal))
  Wire.write(SMPLRT_DIV);
  Wire.write(0x0F);
  Wire.endTransmission();
  delay(5);
  //Setting the clock source : PLL with X axis gyroscope reference
  Wire.beginTransmission(IMU_add);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x01);
  Wire.endTransmission();
  delay(5);
  //Setting the Digital low pass Filter to 0 setting (no filter)
  Wire.beginTransmission(IMU_add);
  Wire.write(CONFIG);
  Wire.write(0x00);
  Wire.endTransmission();  
  delay(5);
  //Setting the Accelerometer Range: +-4g
  Wire.beginTransmission(IMU_add);
  Wire.write(ACCEL_CONFIG);
  Wire.write(0x08);
  Wire.endTransmission();
  delay(5);
  //Setting the Gyro Range: +-250deg/s
  Wire.beginTransmission(IMU_add);
  Wire.write(GYRO_CONFIG);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(5);
}

void mpu6050_getdata(signed char *s, float *input)
{
  Wire.beginTransmission(IMU_add);
  //Burst read 14 registers
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission();

  Wire.requestFrom(IMU_add, 14);

  while(Wire.available() > 14);//Serial.print("Waiting_in_While");
  if(Wire.available() <= 14)
  {
    for( int i = 0; i < 14; i++)
    {
      s[i] = Wire.read();
      //Serial.print("Si_test");
      //Serial.print(s[i]);
      //Serial.print("\n");
    }
  }

  signed int dummy[7];
  dummy[0] = ((int)s[0]<<8 | (int)s[1] );   //ACC X
  dummy[1] = ((int)s[2]<<8 | (int)s[3] );   //ACC Y
  dummy[2] = ((int)s[4]<<8 | (int)s[5] );   //ACC Z
  dummy[3] = ((int)s[6]<<8 | (int)s[7] );   //TEMP
  dummy[4] = ((int)s[8]<<8  | (int)s[9] );  //GYRO X
  dummy[5] = ((int)s[10]<<8 | (int)s[11] ); //GYRO Y
  dummy[6] = ((int)s[12]<<8 | (int)s[13] ); //GYRO Z
  
  //Conver to float
  input[0] = (float)dummy[0]*1.0;
  input[1] = (float)dummy[1]*1.0;
  input[2] = (float)dummy[2]*1.0;
  input[3] = (float)dummy[3]*1.0;
  input[4] = (float)dummy[4]*1.0;
  input[5] = (float)dummy[5]*1.0;
  input[6] = (float)dummy[6]*1.0;
}


//This function gets the initial offsets for the imu. Assuming that imu is flat at the time of calibration
void calibrate_imu(float * off, float * in, signed char * str, int readings)
{
  int i = 0;
  //get average of given values of X-Y-Z gyro and also of X-Y-Z Acc. we can ignore the temp.
  printf("Calibrating IMU...\r\n");
  for( i = 0; i < readings; i++)
  {
    //First get data
    mpu6050_getdata(str, in);
    //now add offsets;
    off[0] += in[0];
    off[1] += in[1];
    off[2] += 1*Acc_sen - in[2] ;   //we want the Acc_z to show 1g.
    off[3] = 0;           //No calibration ofr temp sensor
    off[4] += in[4];
    off[5] += in[5];
    off[6] += in[6];
  }
  //Now divide all and take average. We have our offsets
  for(i = 0; i < 7; i++)
  {
    off[i] /= (readings*1.0);
  } 
}

void process_imu_data(float * process_data, float * raw, float *Off)
{
  int i = 0;
  for(i; i < 7; i++)
  {
    process_data[i] = raw[i] - Off[i];     //Potential Place to add offsets
  }
  //now covert it into readible value by dividing by sensitivity  
  process_data[0] = process_data[0] / Acc_sen; //ACC X
  process_data[1] = process_data[1] / Acc_sen; //ACC Y
  process_data[2] = process_data[2] / Acc_sen; //ACC Z
  
  //For temp sensor, used datasheet and manual...
  process_data[3] = process_data[3] /340.0 + 36.53; //Temp
  
  //we divide by sensitivity
  process_data[4] = process_data[4] / Gyro_sen ; //GYRO X
  process_data[5] = process_data[5] / Gyro_sen ; //GYRO Y
  process_data[6] = process_data[6] / Gyro_sen ; //GYRO Z
}

void initialize_array( float *arr, int width)
{
  for(int i = 0; i < width; i++)
  {
    arr[i] = 0.0;
  }
  Serial.print("\nArray Initailized\n");
  delay(500);
}

float low_pass_filter(float xn, float x1,float y1)
{
  float yn = 0.0;

  yn = y1 * a1 + xn * b0 + x1 * b1;
  return yn;
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.begin(115200);
  Serial.println("\nI2C Scanner");

}

void loop() {
  signed char rbuf[14];
  float raw_data[7];
  float offsets[7];
  float imu_data[7];

  int i;
  float mod_acc;
  float Angle_acc_x;
  float Angle_acc_y;

  float gyro_X = 0.0;
  float gyro_Y = 0.0;
  float gyro_X_prev = 0.0;
  float gyro_Y_prev = 0.0;
  float complementary_angle[1];
  
  double current_time = micros();
  double time_start = 0;
  float time_difference = 0.0;

  //Filter variables
  float yn[1], y1[1], x1[1];

  //Initialize Variables
  initialize_array(yn, 2);
  initialize_array(y1, 2);
  initialize_array(x1, 2);

  //intialize all offsets to 0
  initialize_array(offsets, 7);
  /*for(i = 0; i < 7; i++)
  {
    offsets[i] = 0;
  } 
  */
  initialize_array(complementary_angle, 2);
  
  //Initialize IMU
  mpu6050_init();

  //Initial IMU Calibration
  calibrate_imu(offsets, raw_data, rbuf, 200);

  while(1)
  {
    //Get Raw Data
    mpu6050_getdata(rbuf, raw_data);
    
    //Convert it
    process_imu_data(imu_data, raw_data, offsets);

    //Calculate the Acceleromenter Angles

    mod_acc = sqrt(imu_data[1] * imu_data[1] + imu_data[2] * imu_data[2] + imu_data[0] * imu_data[0]);
    Angle_acc_x = asin(imu_data[1] / mod_acc) * 180.0 / PI;
    Angle_acc_y = asin(imu_data[0] / mod_acc) * -180.0 / PI;

/*
    mod_acc = sqrt(imu_data[1] * imu_data[1] + imu_data[2] * imu_data[2]);
    Angle_acc_y = atan(imu_data[0] / mod_acc) * 180.0 / PI;
    mod_acc = sqrt(imu_data[0] * imu_data[0] + imu_data[2] * imu_data[2]);
    Angle_acc_x = atan(imu_data[1] / mod_acc) * 180.0 / PI;
*/
    //Using LPF on Acceleromenter Data
    yn[0] = low_pass_filter(Angle_acc_x, x1[0], y1[0]);
    x1[0] = Angle_acc_x;
    y1[0] = yn[0];
    y1[1] = low_pass_filter(Angle_acc_y, x1[1], y1[1]);
    x1[1] = Angle_acc_y;
    y1[1] = yn[1];
    
    //Using LPF on Acceleromenter Data
    /*yn[0] = low_pass_filter(imu_data[0], x1[0], y1[0]);
    x1[0] = imu_data[0];
    y1[0] = yn[0];
    y1[1] = low_pass_filter(imu_data[1], x1[1], y1[1]);
    x1[1] = imu_data[1];
    y1[1] = yn[1];
    */
    //oCmpute Gyroscope Angles
    //current_angle(deg) = previous_angle(deg) + speed(deg/sec) * time_b/w_readings(sec)
    current_time = micros();
    time_difference = current_time - time_start;

    time_start = current_time;

    gyro_X = gyro_X_prev + imu_data[4] * time_difference / 1000000.0;
    gyro_Y = gyro_Y_prev + imu_data[5] * time_difference / 1000000.0;     

    //complementary_angle[0] = gyro_X * (complementary_factor) + Angle_acc_x * (1-complementary_factor);
    //complementary_angle[1] = gyro_Y * (complementary_factor) + Angle_acc_y * (1-complementary_factor);

    complementary_angle[0] = gyro_X * (complementary_factor) + yn[0] * (1-complementary_factor);
    complementary_angle[1] = gyro_Y * (complementary_factor) + yn[1] * (1-complementary_factor);    

    gyro_X_prev = complementary_angle[0];
    gyro_Y_prev = complementary_angle[1];
   // Serial.print(imu_data[0]);
    Serial.print(yn[0]);
    //Serial.print(gyro_X);
    Serial.print("\t");
    //Serial.print(gyro_Y);
    Serial.print(Angle_acc_x);
    //Serial.print("\t");
    //Serial.print(yn[0]);
    //Serial.print(complementary_angle[0]);
    Serial.print("\n");
    delay(int(1/fs)*1000);
  }
}
