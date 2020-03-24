// remap scan2scan_node/odom -> camera_odom
// no covariance, no twist
// remap scan2map_node/odom -> lidar_odom
// no covariance, have twist
// remap update_cloud -> point_cloud
//
#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

ros::Publisher vo_odom;
ros::Publisher lidar_odom;
ros::Publisher point_cloud_pub;

bool first_vo = true;
int first_lidar = 0;

bool vo_need_cov = false;
double vo_pos_cov = 0.0;
double vo_vel_cov = 0.0;
bool lidar_need_cov = false;
double lidar_pos_cov = 0.0;
double lidar_vel_cov = 0.0;

ros::Time last_cam_time;
Eigen::Matrix4d last_cam_odom = Eigen::Matrix4d::Identity();
Eigen::Matrix4d first_lidar_trans = Eigen::Matrix4d::Identity();

void vo_odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
   if (first_vo) {
     first_vo = false;
     last_cam_odom.topLeftCorner(3, 3) = Eigen::Quaterniond(msg->pose.pose.orientation.w,
       msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z).toRotationMatrix();
     last_cam_odom.topRightCorner(3, 1) = Eigen::Vector3d(msg->pose.pose.position.x,
                                                           msg->pose.pose.position.y,
                                                           msg->pose.pose.position.z);
     last_cam_time = msg->header.stamp;
     return;
   }

   const double T = (msg->header.stamp - last_cam_time).toSec();
   Eigen::Matrix4d curr_cam_odom = Eigen::Matrix4d::Identity();
   curr_cam_odom.topLeftCorner(3, 3) = Eigen::Quaterniond(msg->pose.pose.orientation.w,
       msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z).toRotationMatrix();
   curr_cam_odom.topRightCorner(3, 1) = Eigen::Vector3d( msg->pose.pose.position.x,
                                                         msg->pose.pose.position.y,
                                                         msg->pose.pose.position.z);

  Eigen::Vector3d linear_diff = curr_cam_odom.topRightCorner<3,1>() - last_cam_odom.topRightCorner<3,1>();
  Eigen::Matrix3d rot_diff = last_cam_odom.topLeftCorner<3,3>().inverse() * curr_cam_odom.topLeftCorner<3,3>();
  tf::Matrix3x3 tf_rot_diff(rot_diff(0,0), rot_diff(0,1), rot_diff(0,2),
                  rot_diff(1,0), rot_diff(1,1), rot_diff(1,2),
                  rot_diff(2,0), rot_diff(2,1), rot_diff(2,2));
  Eigen::Matrix3d curr_rot = curr_cam_odom.topLeftCorner<3,3>();
  tf::Matrix3x3 tf_curr_rot(curr_rot(0,0), curr_rot(0,1), curr_rot(0,2),
                            curr_rot(1,0), curr_rot(1,1), curr_rot(1,2),
                            curr_rot(2,0), curr_rot(2,1), curr_rot(2,2));
  Eigen::Matrix3d last_rot = last_cam_odom.topLeftCorner<3,3>();
  tf::Matrix3x3 tf_last_rot(last_rot(0,0), last_rot(0,1), last_rot(0,2),
                            last_rot(1,0), last_rot(1,1), last_rot(1,2),
                            last_rot(2,0), last_rot(2,1), last_rot(2,2));
  double curr_roll, curr_pitch, curr_yaw;
  tf_curr_rot.getRPY(curr_roll, curr_pitch, curr_yaw);
  double last_roll, last_pitch, last_yaw;
  tf_last_rot.getRPY(last_roll, last_pitch, last_yaw);
  double roll, pitch, yaw;
  tf_rot_diff.getRPY(roll, pitch, yaw);

  Eigen::Matrix<double, 3, 5> rpy;
  rpy << curr_roll, last_roll, curr_roll - last_roll, roll, curr_roll - last_roll - roll,
         curr_pitch, last_pitch, curr_pitch - last_pitch, pitch, curr_pitch - last_pitch - pitch,
         curr_yaw, last_yaw, curr_yaw - last_yaw, yaw, curr_yaw - last_yaw - yaw;
//  std::cout << rpy << std::endl;
  if (std::abs(curr_roll - last_roll - roll) > 0.1 || std::abs(curr_pitch - last_pitch - pitch) > 0.1 || std::abs(curr_yaw - last_yaw - yaw) > 0.1)
    ROS_WARN("Roll Pitch Yaw differential has precision issue.");

  nav_msgs::Odometry modified_msg = *msg;
  modified_msg.header.stamp = ros::Time::now();
  modified_msg.header.frame_id = "odom";
  modified_msg.child_frame_id = "base_link";

  // modified_msg.pose.pose.position.x = curr_wrt_first(0,3);
  // modified_msg.pose.pose.position.y = curr_wrt_first(1,3);
  // modified_msg.pose.pose.position.z = curr_wrt_first(2,3);
  // modified_msg.pose.pose.orientation.w = curr_wrt_first_q.w();
  // modified_msg.pose.pose.orientation.x = curr_wrt_first_q.x();
  // modified_msg.pose.pose.orientation.y = curr_wrt_first_q.y();
  // modified_msg.pose.pose.orientation.z = curr_wrt_first_q.z();

  modified_msg.twist.twist.linear.x = linear_diff(0)/T;
  modified_msg.twist.twist.linear.y = linear_diff(1)/T;
  modified_msg.twist.twist.linear.z = linear_diff(2)/T;
  modified_msg.twist.twist.angular.x = (curr_roll - last_roll)/T;
  modified_msg.twist.twist.angular.y = (curr_pitch - last_pitch)/T;
  modified_msg.twist.twist.angular.z = (curr_yaw - last_yaw)/T;

  if (vo_need_cov) {
    modified_msg.pose.covariance = {vo_pos_cov, 0., 0., 0., 0., 0.,
                                    0., vo_pos_cov, 0., 0., 0., 0.,
                                    0., 0., vo_pos_cov, 0., 0., 0.,
                                    0., 0., 0., vo_pos_cov, 0., 0.,
                                    0., 0., 0., 0., vo_pos_cov, 0.,
                                    0., 0., 0., 0., 0., vo_pos_cov};
    modified_msg.pose.covariance = {vo_vel_cov, 0., 0., 0., 0., 0.,
                                    0., vo_vel_cov, 0., 0., 0., 0.,
                                    0., 0., vo_vel_cov, 0., 0., 0.,
                                    0., 0., 0., vo_vel_cov, 0., 0.,
                                    0., 0., 0., 0., vo_vel_cov, 0.,
                                    0., 0., 0., 0., 0., vo_vel_cov};
  }

  vo_odom.publish(modified_msg);
}

void lidar_odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  // if (first_lidar < 5) {
  //   first_lidar ++;
  //   first_lidar_pos = Eigen::Vector3d(msg->pose.pose.position.x,
  //                                     msg->pose.pose.position.y,
  //                                     msg->pose.pose.position.z);
  //   return;
  // }

  nav_msgs::Odometry modified_msg = *msg;
  modified_msg.header.stamp = ros::Time::now();
  modified_msg.header.frame_id = "odom";
  modified_msg.child_frame_id = "base_link";
  if (lidar_need_cov) {
    modified_msg.pose.covariance = {lidar_pos_cov, 0., 0., 0., 0., 0.,
                                    0., lidar_pos_cov, 0., 0., 0., 0.,
                                    0., 0., lidar_pos_cov, 0., 0., 0.,
                                    0., 0., 0., lidar_pos_cov, 0., 0.,
                                    0., 0., 0., 0., lidar_pos_cov, 0.,
                                    0., 0., 0., 0., 0., lidar_pos_cov};
    modified_msg.twist.covariance = {lidar_vel_cov, 0., 0., 0., 0., 0.,
                                    0., lidar_vel_cov, 0., 0., 0., 0.,
                                    0., 0., lidar_vel_cov, 0., 0., 0.,
                                    0., 0., 0., lidar_vel_cov, 0., 0.,
                                    0., 0., 0., 0., lidar_vel_cov, 0.,
                                    0., 0., 0., 0., 0., lidar_vel_cov};
  }
  lidar_odom.publish(modified_msg);
}

void point_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  sensor_msgs::PointCloud2 modified_msg = *msg;
  modified_msg.header.frame_id = "lidar";
  point_cloud_pub.publish(modified_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "TF_Odom");

  ros::NodeHandle node;
  ros::NodeHandle private_node("~");

  private_node.getParam("vo_need_cov", vo_need_cov);
  private_node.getParam("lidar_need_cov", lidar_need_cov);
  private_node.getParam("vo_pos_cov", vo_pos_cov);
  private_node.getParam("vo_vel_cov", vo_vel_cov);
  private_node.getParam("lidar_pos_cov", lidar_pos_cov);
  private_node.getParam("lidar_vel_cov", lidar_vel_cov);

  vo_odom = node.advertise<nav_msgs::Odometry>("camera_odom", 10);
  lidar_odom = node.advertise<nav_msgs::Odometry>("lidar_odom", 10);
  point_cloud_pub = node.advertise<sensor_msgs::PointCloud2>("point_cloud", 10);

  ros::Subscriber vo_odom_listener = node.subscribe("scan2scan_node/odom", 100, vo_odom_callback);
  ros::Subscriber lidar_odom_listener = node.subscribe("scan2map_node/odom", 100, lidar_odom_callback);
  ros::Subscriber point_cloud_listener = node.subscribe("fusion_node/update_cloud", 10, point_cloud_callback);

  ros::spin();
  return 0;
}
