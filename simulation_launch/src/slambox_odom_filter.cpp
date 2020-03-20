#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher vo_odom;
ros::Publisher lidar_odom;
ros::Publisher point_cloud_pub;

bool first_vo = true;
int first_lidar = 0;

bool vo_need_cov = false;
double vo_pos_cov = 0.0;
bool lidar_need_cov = false;
double lidar_pos_cov = 0.0;
double lidar_vel_cov = 0.0;

Eigen::Matrix4d first_cam_odom = Eigen::Matrix4d::Identity();
Eigen::Matrix4d first_lidar_trans = Eigen::Matrix4d::Identity();

void vo_odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
   if (first_vo) {
     first_vo = false;
     first_cam_odom.topLeftCorner(3, 3) = Eigen::Quaterniond(msg->pose.pose.orientation.w,
       msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z).toRotationMatrix();
     first_cam_odom.topRightCorner(3, 1) = Eigen::Vector3d(msg->pose.pose.position.x,
                                                           msg->pose.pose.position.y,
                                                           msg->pose.pose.position.z);
     return;
   }

  // Eigen::Matrix4d curr_cam_odom = Eigen::Matrix4d::Identity();
  // curr_cam_odom.topLeftCorner(3, 3) = Eigen::Quaterniond(msg->pose.pose.orientation.w, 
  //     msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z).toRotationMatrix();
  // curr_cam_odom.topRightCorner(3, 1) = Eigen::Vector3d( msg->pose.pose.position.x,
  //                                                       msg->pose.pose.position.y,
  //                                                       msg->pose.pose.position.z);
  
  // Eigen::Matrix4d curr_wrt_first = first_cam_odom.inverse() * curr_cam_odom;
  // Eigen::Matrix3d curr_wrt_first_R = curr_wrt_first.topLeftCorner(3, 3);
  // // Eigen::Matrix3d init_guess_for_odom_frame_yaw = Eigen::AngleAxisd(init_odom_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  // Eigen::Matrix3d init_guess_for_odom_frame_yaw = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  // Eigen::Quaterniond curr_wrt_first_q(curr_wrt_first_R * init_guess_for_odom_frame_yaw);

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
  if (vo_need_cov) {
    modified_msg.pose.covariance = {vo_pos_cov, 0., 0., 0., 0., 0.,
                                    0., vo_pos_cov, 0., 0., 0., 0.,
                                    0., 0., vo_pos_cov, 0., 0., 0.,
                                    0., 0., 0., vo_pos_cov, 0., 0.,
                                    0., 0., 0., 0., vo_pos_cov, 0.,
                                    0., 0., 0., 0., 0., vo_pos_cov};
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
  private_node.getParam("lidar_pos_cov", lidar_pos_cov);
  private_node.getParam("lidar_vel_cov", lidar_vel_cov);

  vo_odom = node.advertise<nav_msgs::Odometry>("camera_odom", 10);
  lidar_odom = node.advertise<nav_msgs::Odometry>("lidar_odom", 10);
  point_cloud_pub = node.advertise<sensor_msgs::PointCloud2>("point_cloud", 10);

  ros::Subscriber vo_odom_listener = node.subscribe("scan2scan_odom", 100, vo_odom_callback);
  ros::Subscriber lidar_odom_listener = node.subscribe("aft_mapped_to_init", 100, lidar_odom_callback);
  ros::Subscriber point_cloud_listener = node.subscribe("update_cloud", 10, point_cloud_callback);

  ros::spin();
  return 0;
}
