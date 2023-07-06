/*
 * Author: Automatic Addison
 * Website: https://automaticaddison.com
 * Description: ROS node that publishes the accumulated ticks for each wheel
 * (right_ticks and left_ticks topics) using the built-in encoder 
 * (forward = positive; reverse = negative) 
 */

#include "MPU9250.h"
#include <ros.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Imu.h>

// Handles startup and shutdown of ROS
ros::NodeHandle nh;

// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 2
#define ENC_IN_RIGHT_A 19

// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 3
#define ENC_IN_RIGHT_B 18

// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;

// Minumum and maximum values for 16-bit integers
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

// Keep track of the number of wheel ticks
std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);

std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

// Keep track of mpu9250 imu sensor
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu_data", &imu_msg);
MPU9250 imu;

// 100ms interval for measurements
const int interval = 100;
long previousMillis = 0;
long currentMillis = 0;

//imu sensor reading
void print_roll_pitch_yaw() {
  // Baca yaw, pitch, dan roll dari MPU9250
  float yaw = imu.getYaw();      // Mendapatkan nilai yaw menggunakan fungsi getYaw()
  float pitch = imu.getPitch();  // Mendapatkan nilai pitch menggunakan fungsi getPitch()
  float roll = imu.getRoll();    // Mendapatkan nilai roll menggunakan fungsi getRoll()

  // Baca kecepatan sudut (angular velocity) dari MPU9250
  float angular_velocity_x = imu.getGyroX();  // Mendapatkan nilai kecepatan sudut pada sumbu x menggunakan fungsi getGyroX()
  float angular_velocity_y = imu.getGyroY();  // Mendapatkan nilai kecepatan sudut pada sumbu y menggunakan fungsi getGyroY()
  float angular_velocity_z = imu.getGyroZ();  // Mendapatkan nilai kecepatan sudut pada sumbu z menggunakan fungsi getGyroZ()

  // Baca percepatan linier (linear acceleration) dari MPU9250
  float linear_acceleration_x = imu.getLinearAccX();  // Mendapatkan nilai percepatan linier pada sumbu x menggunakan fungsi getLinearAccX()
  float linear_acceleration_y = imu.getLinearAccY();  // Mendapatkan nilai percepatan linier pada sumbu y menggunakan fungsi getLinearAccY()
  float linear_acceleration_z = imu.getLinearAccZ();  // Mendapatkan nilai percepatan linier pada sumbu z menggunakan fungsi getLinearAccZ()

  // Isi data IMU
  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id = "imu_link";
  // Konversi Euler angles menjadi quaternion
  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);

  float qw = cy * cp * cr + sy * sp * sr;
  float qx = cy * cp * sr - sy * sp * cr;
  float qy = sy * cp * sr + cy * sp * cr;
  float qz = sy * cp * cr - cy * sp * sr;

  // Isi data quaternion ke pesan IMU
  imu_msg.orientation.x = qx;
  imu_msg.orientation.y = qy;
  imu_msg.orientation.z = qz;
  imu_msg.orientation.w = qw;

  imu_msg.angular_velocity.x = angular_velocity_x;
  imu_msg.angular_velocity.y = angular_velocity_y;
  imu_msg.angular_velocity.z = angular_velocity_z;

  imu_msg.linear_acceleration.x = linear_acceleration_x;
  imu_msg.linear_acceleration.y = linear_acceleration_y;
  imu_msg.linear_acceleration.z = linear_acceleration_z;
  // Serial.print("Yaw, Pitch, Roll: ");
  // Serial.print(yaw, 2);
  // Serial.print(", ");
  // Serial.print(pitch, 2);
  // Serial.print(", ");
  // Serial.println(roll, 2);
}

// Increment the number of ticks
void right_wheel_tick() {

  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);

  if (val == LOW) {
    Direction_right = false;  // Reverse
  } else {
    Direction_right = true;  // Forward
  }

  if (Direction_right) {

    if (right_wheel_tick_count.data == encoder_maximum) {
      right_wheel_tick_count.data = encoder_minimum;
    } else {
      right_wheel_tick_count.data++;
    }
  } else {
    if (right_wheel_tick_count.data == encoder_minimum) {
      right_wheel_tick_count.data = encoder_maximum;
    } else {
      right_wheel_tick_count.data--;
    }
  }
}

// Increment the number of ticks
void left_wheel_tick() {

  // Read the value for the encoder for the left wheel
  int val = digitalRead(ENC_IN_LEFT_B);

  if (val == LOW) {
    Direction_left = true;  // Reverse
  } else {
    Direction_left = false;  // Forward
  }

  if (Direction_left) {
    if (left_wheel_tick_count.data == encoder_maximum) {
      left_wheel_tick_count.data = encoder_minimum;
    } else {
      left_wheel_tick_count.data++;
    }
  } else {
    if (left_wheel_tick_count.data == encoder_minimum) {
      left_wheel_tick_count.data = encoder_maximum;
    } else {
      left_wheel_tick_count.data--;
    }
  }
}

void setup() {
  //set i2c connection from mpu9250
  Wire.begin();
  delay(2000);
  if (!imu.setup(0x68)) {  // change to your own address
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }

  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B, INPUT);
  pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B, INPUT);

  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);

  // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.advertise(imu_pub);
}

void loop() {
  //update mpu value if there any change
  if (imu.update()) {
    static uint32_t prev_ms = millis();
    if (millis() > prev_ms + 25) {
      print_roll_pitch_yaw();
      prev_ms = millis();
    }
  }

  // Record the time
  currentMillis = millis();

  // If 100ms have passed, print the number of ticks
  if (currentMillis - previousMillis > interval) {

    previousMillis = currentMillis;

    rightPub.publish(&right_wheel_tick_count);
    leftPub.publish(&left_wheel_tick_count);
    imu_pub.publish(&imu_msg);
    nh.spinOnce();
  }
}