#include <Arduino.h>
#include <ros.h>
#include <mpu9250_angacc.hpp>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <LPfilter.hpp>
#include <MadgwickAHRS.h>
#include <MPU9250_Qavairable.hpp>


ros::NodeHandle nh;

sensor_msgs::Imu msg_imu;
geometry_msgs::Vector3Stamped msg_ang_acc;
geometry_msgs::Vector3Stamped msg_ang_acc_raw;

ros::Publisher responser_ang_acc("angular_acc", &msg_ang_acc);
ros::Publisher responser_ang_acc_raw("angular_acc_raw", &msg_ang_acc_raw);
ros::Publisher responser_imu("imu1", &msg_imu);

MPU9250_ANGACC Imu(Wire,0x68);
Madgwick filter;

LPfilter ang_vel_filter[] = {
  LPfilter(0.002,10),
  LPfilter(0.002,10),
  LPfilter(0.002,10)
};

LPfilter ang_acc_filter[] = {
  LPfilter(0.002,10),
  LPfilter(0.002,10),
  LPfilter(0.002,10)
};
int prevmicros = 0;
int IMUUpdateThreadTimer = 0;
int loopInterval = 0;

void IMUUpdate();
void publishData();

void setup() {
  // put your setup code here, to run once:
  while(!Serial);
  Serial.begin(115200);
  Imu.setupIMU();
  Imu.setAccelRange(MPU9250::ACCEL_RANGE_16G);
  Imu.setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  Imu.setSrd(1); // 500Hz

  filter.begin(500);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(responser_ang_acc);
  nh.advertise(responser_ang_acc_raw);
  nh.advertise(responser_imu);
}

void loop() {
  // put your main code here, to run repeatedly:
  int curmicros = micros();

  IMUUpdate();
  publishData();

  nh.spinOnce();
  while(micros() - curmicros<2000);
  loopInterval = micros() - prevmicros;
  // Serial.println(loopInterval);
  prevmicros = micros();
}

void IMUUpdate() {
  // if (micros() - IMUUpdateThreadTimer < 5000) return;
  int start = micros();
  // update imu
  Imu.readSensor();
  Imu.computeAngularAccel();
  // get value from senser class 
  // Serial.println(micros() - IMUUpdateThreadTimer);
  msg_imu.header.stamp = nh.now();
  vector<float> angular_vel = Imu.getAngularVelRPS();
  msg_imu.angular_velocity.x = ang_vel_filter[0].updateVal(angular_vel[1]);
  msg_imu.angular_velocity.y = ang_vel_filter[1].updateVal(angular_vel[2]);
  msg_imu.angular_velocity.z = ang_vel_filter[2].updateVal(-angular_vel[0]);
  // vector<float> linear_acc = Imu.getLinearAccel();
  // msg_imu.linear_acceleration.x = linear_acc[0];
  // msg_imu.linear_acceleration.y = linear_acc[1];
  // msg_imu.linear_acceleration.z = linear_acc[2];
  vector<float> ang_acc_5point = Imu.getAngularAccelRPSS_5point();
  msg_ang_acc.vector.x = ang_acc_filter[0].updateVal(ang_acc_5point[1]);
  msg_ang_acc.vector.y = ang_acc_filter[1].updateVal(ang_acc_5point[2]);
  msg_ang_acc.vector.z = ang_acc_filter[2].updateVal(-ang_acc_5point[0]);
  msg_ang_acc_raw.vector.x = ang_acc_5point[1];
  msg_ang_acc_raw.vector.y = ang_acc_5point[2];
  msg_ang_acc_raw.vector.z = -ang_acc_5point[0];
  // vector<float> quaternion = Imus[0].getQuaternion();
  // msg_imu.orientation.w = quaternion[0];
  // msg_imu.orientation.x = quaternion[1];
  // msg_imu.orientation.y = quaternion[2];
  // msg_imu.orientation.z = quaternion[3];
  msg_ang_acc.header.stamp = nh.now();
  msg_ang_acc_raw.header.stamp = nh.now();    

  // calucrate quaternion by madgwick filter
  filter.update(Imu.getGyroX_rads(),Imu.getGyroY_rads(),Imu.getGyroZ_rads(),
                 Imu.getAccelX_mss(),Imu.getAccelY_mss(),Imu.getAccelZ_mss(),
                    Imu.getMagX_uT(),Imu.getMagY_uT(),Imu.getMagZ_uT());

  IMUUpdateThreadTimer = micros();
}

void publishData() {
  // need to 5ms wait for avoiding serial clush.
  // if (micros() - PublishDataThreadTimer < 5000) return;
  responser_ang_acc.publish( &msg_ang_acc );
  responser_ang_acc_raw.publish( &msg_ang_acc_raw );
  responser_imu.publish( &msg_imu );
}