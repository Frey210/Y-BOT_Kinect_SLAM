#include "MPU9250.h"
#include <ros.h>
#include <sensor_msgs/Imu.h>

ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;

ros::Publisher imu_pub("imu_data", &imu_msg);

MPU9250 mpu;

void setup() {
  nh.initNode();
  nh.advertise(imu_pub);
  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  if (!mpu.setup(0x68)) {  // change to your own address
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }
}

void loop() {
  if (mpu.update()) {
    static uint32_t prev_ms = millis();
    if (millis() > prev_ms + 25) {
      print_roll_pitch_yaw();
      imu_pub.publish(&imu_msg);
      nh.spinOnce();
      prev_ms = millis();
    }
  }
}

void print_roll_pitch_yaw() {
  float yaw = mpu.getYaw();
  float pitch = mpu.getPitch();
  float roll = mpu.getRoll();
  imu_msg.header.stamp = nh.now();
  imu_msg.orientation.x = roll;
  imu_msg.orientation.y = pitch;
  imu_msg.orientation.z = yaw;
  Serial.print("Yaw, Pitch, Roll: ");
  Serial.print(yaw, 2);
  Serial.print(", ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
}
