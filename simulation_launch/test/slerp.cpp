//
// Created by gzy on 3/20/20.
//

#include <Eigen/Dense>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Imu.h>

int main (int argc, char** argv) {
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(10);

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Publisher imu_pub1 = n.advertise<sensor_msgs::Imu>("marker1", 1);
  ros::Publisher imu_pub2 = n.advertise<sensor_msgs::Imu>("marker2", 1);
  ros::Publisher imu_pub3 = n.advertise<sensor_msgs::Imu>("marker3", 1);

  Eigen::Quaterniond q1(1,2,3,4); q1.normalize();
  Eigen::Quaterniond q2(3,-5,6,-7); q2.normalize();

  double i = 0;
  double di = 0.01;
  while (ros::ok()) {
    sensor_msgs::Imu msg1;
    msg1.header.frame_id = "left";
    msg1.header.stamp = ros::Time::now();
    msg1.orientation.w = q1.w();
    msg1.orientation.x = q1.x();
    msg1.orientation.y = q1.y();
    msg1.orientation.z = q1.z();
    imu_pub1.publish(msg1);

    sensor_msgs::Imu msg2;
    msg2.header.frame_id = "right";
    msg2.header.stamp = ros::Time::now();
    msg2.orientation.w = q2.w();
    msg2.orientation.x = q2.x();
    msg2.orientation.y = q2.y();
    msg2.orientation.z = q2.z();
    imu_pub2.publish(msg2);

    i += di;
    if (i >= 1) {
      i = 1;
      di = -di;
    }
    if (i <= 0) {
      i = 0;
      di = -di;
    }
    Eigen::Quaterniond q3 = q1.slerp(i, q2);

    sensor_msgs::Imu msg3;
    msg3.header.frame_id = "map";
    msg3.header.stamp = ros::Time::now();
    msg3.orientation.w = q3.w();
    msg3.orientation.x = q3.x();
    msg3.orientation.y = q3.y();
    msg3.orientation.z = q3.z();
    imu_pub3.publish(msg3);

    r.sleep();
  }
  return 0;
}