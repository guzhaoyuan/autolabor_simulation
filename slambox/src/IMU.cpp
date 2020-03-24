#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>

ros::Publisher IMU_publisher;
double ori_cov = 0.0;
int first_imu = 0;
Eigen::Quaterniond q_first;

void IMU_callback(const sensor_msgs::Imu::ConstPtr& msg) {
  if (first_imu < 5) {
    q_first = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    first_imu ++;
    return;
  }

  sensor_msgs::Imu imu_data = *msg;
  imu_data.header.frame_id = "base_link";
  imu_data.header.stamp = ros::Time().now();
  imu_data.orientation_covariance = {ori_cov, 0., 0.,
                                     0., ori_cov, 0.,
                                     0., 0., ori_cov};

  Eigen::Quaterniond q_now(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  Eigen::Quaterniond q_diff = q_first.inverse() * q_now;

  imu_data.orientation.w = q_diff.w();
  imu_data.orientation.x = q_diff.x();
  imu_data.orientation.y = q_diff.y();
  imu_data.orientation.z = q_diff.z();

  IMU_publisher.publish(imu_data);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "IMU");
    ros::NodeHandle node;

    ros::NodeHandle private_node("~");
    private_node.getParam("ori_cov", ori_cov);

    ros::Subscriber IMU_listener = node.subscribe("imu/data", 100, IMU_callback);
    IMU_publisher = node.advertise<sensor_msgs::Imu>("imu_data", 10);
    ros::spin();

    return 0;
}
