// https://atadiat.com/en/e-ros-imu-and-arduino-how-to-send-to-ros/ : use string !
#include "Wire.h"

#define PIN_TRIGGER 7
#define gravity 9.80655

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature; // variables for temperature data

// ROS related 
#include <ros.h>
#include <std_msgs/String.h>

// node handler
std_msgs::String imu_msg;
ros::Publisher pub_imu("/imu/mpu_6050", &imu_msg);
ros::NodeHandle nh;

void setup() {
  pinMode(PIN_TRIGGER, OUTPUT);
  
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000L);
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x00);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  // 0: +-2g (16384 LSB/g), 1: 4g 8192, 2: 8g 4096, 3: 16g 2048
  Wire.endTransmission(true);
  
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  // 0: 250 deg/s (131), 1: 500 deg/s (65.5), 2: 1000 deg/s (32.8), 3: 2000 deg/s (16.4)
  Wire.endTransmission(true);
  
  // Configure interrupt pin
  Wire.beginTransmission(MPU_ADDR);
//  Wire.write(0x37); //Talk to the INT_PIN_CFG
 // Wire.write(0b00000000);
  //Wire.write(0x38); //
  //Wire.write(0b00000001);
  Wire.endTransmission(true);

  // ROS initialization
    nh.getHardware()->setBaud(115200);

  nh.initNode();
  nh.advertise(pub_imu);
}

int cnt = 1;
long publisher_timer;

void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accelerometer_x = (Wire.read()<<8 | Wire.read()); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = (Wire.read()<<8 | Wire.read()); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = (Wire.read()<<8 | Wire.read()); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)

  temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x      = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y      = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z      = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  
  // count (200 Hz IMU. 20 Hz image)
  cnt++;
  if(cnt > 10){
    cnt = 1;
    digitalWrite(PIN_TRIGGER, HIGH);
    digitalWrite(PIN_TRIGGER, LOW);
  }
  // print out data
  
  /*Serial.print("aX = "); Serial.print(acc[0]);
  Serial.print(" | aY = "); Serial.print(acc[1]);
  Serial.print(" | aZ = "); Serial.print(acc[2]);
  // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
  Serial.print(" | tmp = "); Serial.print(temperature/340.00+36.53);
  Serial.print(" | gX = "); Serial.print(gyro[0]);
  Serial.print(" | gY = "); Serial.print(gyro[1]);
  Serial.print(" | gZ = "); Serial.print(gyro[2]);
  Serial.println();*/

  String AX = String(accelerometer_x);
  String AY = String(accelerometer_y);
  String AZ = String(accelerometer_z);
  String GX = String(gyro_x);
  String GY = String(gyro_y);
  String GZ = String(gyro_z);
  String tmp = String(temperature);
 
  String data = "A" + AX + "B"+ AY + "C" + AZ + "D" + GX + "E" + GY + "F" + GZ + "G" ;
  Serial.println(data);
  int length = data.indexOf("G") +2;
  char data_final[length+1];
  data.toCharArray(data_final, length+1);

  if(millis() > publisher_timer){
    // step 1: request reading from sensor
    imu_msg.data = data_final;
    pub_imu.publish(&imu_msg);
    publisher_timer = millis(); // 200 Hz 
    nh.spinOnce();
  }
  
  // delay 5 ms (for 200 Hz)
  
}
