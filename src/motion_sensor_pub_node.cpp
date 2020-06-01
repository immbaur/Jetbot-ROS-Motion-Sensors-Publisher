#include <iostream>
#include <thread>

#include <jetbot_ros_communication_package/Rpm.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <RTIMULib.h>
#include <rotary-encoder/rotary_encoder.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "jetbot_ros_motion_sensors_publisher");

  ros::NodeHandle n;

  ros::Publisher raw_imu_pub = n.advertise<sensor_msgs::Imu>("raw_imu_pub", 1);
  ros::Publisher raw_rpm_pub = n.advertise<jetbot_ros_communication_package::Rpm>("raw_rpm_pub", 1);
  ros::Publisher moving_average_rpm_pub = n.advertise<jetbot_ros_communication_package::Rpm>("moving_average_rpm_pub", 1);

  sensor_msgs::Imu raw_imu_msg;
  jetbot_ros_communication_package::Rpm raw_rpm_msg;
  jetbot_ros_communication_package::Rpm moving_average_rpm_msg;

  RTIMUSettings *settings = new RTIMUSettings("RTIMULib");
  RTIMU *imu = RTIMU::createIMU(settings);

  if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL)) {
    printf("No IMU found\n");
    exit(1);
  }

  imu->IMUInit();
  imu->setSlerpPower(0.02);
  imu->setGyroEnable(true);
  imu->setAccelEnable(true);
  imu->setCompassEnable(true);

  RotaryEncoder rotaryEncoderRight(50, 48 * 4, 5.2 * 10 * 1000);
  RotaryEncoder rotaryEncoderLeft(14, 48 * 4, 5.2 * 10 * 1000);

  ros::Rate loop_rate(50);

  while (ros::ok()) {
    if (imu->IMURead()) {
      // std::cout << "." << std::endl;
      RTIMU_DATA imuData = imu->getIMUData();
      // printf("Sample rate %d: %s\r", 0, RTMath::displayDegrees("", imuData.fusionPose));
      // fflush(stdout);
      ros::Time current_time = ros::Time::now();

      raw_imu_msg.header.stamp = current_time; // TODO check if timestamp from library makes more sense
      raw_imu_msg.angular_velocity.x = imuData.gyro.x();
      raw_imu_msg.angular_velocity.y = imuData.gyro.y();
      raw_imu_msg.angular_velocity.z = imuData.gyro.z();

      raw_imu_msg.linear_acceleration.x = imuData.accel.x();
      raw_imu_msg.linear_acceleration.y = imuData.accel.y();
      raw_imu_msg.linear_acceleration.z = imuData.accel.z();

      raw_imu_msg.orientation.x = imuData.compass.x();
      raw_imu_msg.orientation.y = imuData.compass.y();
      raw_imu_msg.orientation.z = imuData.compass.z();

      raw_imu_pub.publish(raw_imu_msg);

    } else {
      std::cout << "skipped" << std::endl;
    }
    raw_rpm_msg.header.stamp = ros::Time::now();
    raw_rpm_msg.right = rotaryEncoderRight.rpm(0);
    raw_rpm_msg.left = rotaryEncoderLeft.rpm(0);

    moving_average_rpm_msg.header.stamp = ros::Time::now();
    moving_average_rpm_msg.right = rotaryEncoderRight.rpm(3);
    moving_average_rpm_msg.left = rotaryEncoderLeft.rpm(3);

    raw_rpm_pub.publish(raw_rpm_msg);
    moving_average_rpm_pub.publish(moving_average_rpm_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
