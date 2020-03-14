#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>

class OdomFilter {
 public:
  OdomFilter() {

    ros::NodeHandle private_node("~");

    private_node.getParam("vo_pos_cov", vo_pos_cov);
    private_node.getParam("vo_vel_cov", vo_pos_cov);
    private_node.getParam("uwb_pos_cov", uwb_pos_cov);
    private_node.getParam("init_odom_yaw", init_odom_yaw);
    private_node.getParam("init_imu_yaw", init_imu_yaw);
    private_node.getParam("imu_ori_cov", imu_ori_cov);
    private_node.getParam("distance_UWB_to_base", distance_UWB_to_base);

    vo_odom = node.advertise<nav_msgs::Odometry>("t265/odom", 10);
    uwb_odom = node.advertise<nav_msgs::Odometry>("dwm/odom", 10);
    imu_odom = node.advertise<sensor_msgs::Imu>("imu/odom", 10);



  }

  void vo_odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (first_vo) {
      first_vo = false;
//    first_cam_odom.topLeftCorner(3, 3) = Eigen::Quaterniond(msg->pose.pose.orientation.w,
//      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z).toRotationMatrix();
//    first_cam_odom.topRightCorner(3, 1) = Eigen::Vector3d(msg->pose.pose.position.x,
//                                                          msg->pose.pose.position.y,
//                                                          msg->pose.pose.position.z);
      return;
    }

    Eigen::Matrix4d curr_cam_odom = Eigen::Matrix4d::Identity();
    curr_cam_odom.topLeftCorner(3, 3) = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                                           msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z).toRotationMatrix();
    curr_cam_odom.topRightCorner(3, 1) = Eigen::Vector3d( msg->pose.pose.position.x,
                                                          msg->pose.pose.position.y,
                                                          msg->pose.pose.position.z);

    Eigen::Matrix4d curr_wrt_first = first_cam_odom.inverse() * curr_cam_odom;
    Eigen::Matrix3d curr_wrt_first_R = curr_wrt_first.topLeftCorner(3, 3);
    // Eigen::Matrix3d init_guess_for_odom_frame_yaw = Eigen::AngleAxisd(init_odom_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix3d init_guess_for_odom_frame_yaw = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Quaterniond curr_wrt_first_q(curr_wrt_first_R * init_guess_for_odom_frame_yaw);

    nav_msgs::Odometry modified_msg = *msg;
    modified_msg.header.stamp = ros::Time::now();
    modified_msg.header.frame_id = "odom";
    modified_msg.child_frame_id = "base_link";

    modified_msg.pose.pose.position.x = curr_wrt_first(0,3);
    modified_msg.pose.pose.position.y = curr_wrt_first(1,3);
    modified_msg.pose.pose.position.z = 0;
    modified_msg.pose.pose.orientation.w = curr_wrt_first_q.w();
    modified_msg.pose.pose.orientation.x = curr_wrt_first_q.x();
    modified_msg.pose.pose.orientation.y = curr_wrt_first_q.y();
    modified_msg.pose.pose.orientation.z = curr_wrt_first_q.z();

    modified_msg.pose.covariance = {vo_pos_cov, 0., 0., 0., 0., 0.,
                                    0., vo_pos_cov, 0., 0., 0., 0.,
                                    0., 0., 0., 0., 0., 0.,
                                    0., 0., 0., 0., 0., 0.,
                                    0., 0., 0., 0., 0., 0.,
                                    0., 0., 0., 0., 0., vo_pos_cov};

    modified_msg.twist.covariance = {vo_vel_cov, 0., 0., 0., 0., 0.,
                                     0., vo_vel_cov, 0., 0., 0., 0.,
                                     0., 0., 0., 0., 0., 0.,
                                     0., 0., 0., 0., 0., 0.,
                                     0., 0., 0., 0., 0., 0.,
                                     0., 0., 0., 0., 0., vo_vel_cov};
    vo_odom.publish(modified_msg);
  }

  void uwb_odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (first_uwb < 5) {
      first_uwb ++;
      first_uwb_pos = Eigen::Vector3d(msg->pose.pose.position.x,
                                      msg->pose.pose.position.y,
                                      msg->pose.pose.position.z);
      return;
    }

    nav_msgs::Odometry modified_msg = *msg;
    modified_msg.header.stamp = ros::Time::now();
    modified_msg.header.frame_id = "uwb_odom";
    modified_msg.child_frame_id = "base_link";

    // Try to rotate the UWB sensor reading to align with the map frame.
    // Assume the first UWB reading is at the odom frame. To make alignment, the initial odom frame angle is guessed.
    Eigen::Vector3d position_W = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z) - first_uwb_pos;
    Eigen::Vector3d position_UWB = Eigen::AngleAxisd(init_odom_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
        position_W;

    tf::StampedTransform odom_to_base_link;
    try{
      listener.lookupTransform("/base_link", "/odom",
                               ros::Time(0), odom_to_base_link);
    }
    catch (tf::TransformException ex){
      odom_to_base_link.setIdentity();
    }
    tf::Quaternion q = odom_to_base_link.getRotation();
    Eigen::Quaterniond q_WO(q.w(),q.x(),q.y(),q.z());
    Eigen::Matrix3d R_WO = q_WO.toRotationMatrix();
    // Compute the base_link in odom frame by adding offset from UWB.
    Eigen::Vector3d position_base = position_UWB + R_WO.transpose() * Eigen::Vector3d(distance_UWB_to_base, 0, 0);

    modified_msg.pose.pose.position.x = position_base(0);
    modified_msg.pose.pose.position.y = position_base(1);
    modified_msg.pose.pose.position.z = 0;

    modified_msg.pose.covariance = {uwb_pos_cov, 0., 0., 0., 0., 0.,
                                    0., uwb_pos_cov, 0., 0., 0., 0.,
                                    0., 0., 0., 0., 0., 0.,
                                    0., 0., 0., 0., 0., 0.,
                                    0., 0., 0., 0., 0., 0.,
                                    0., 0., 0., 0., 0., 0.};

    uwb_odom.publish(modified_msg);
  }

  double GetEulerZ (const sensor_msgs::Imu::ConstPtr& msg) {
    Eigen::Quaterniond q_origin(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

    auto euler = q_origin.toRotationMatrix().eulerAngles(2, 0, 2);
    ROS_INFO("The angles are: %.2f, %.2f, %.2f", euler[0],euler[1],euler[2]);

    return euler[0];
  }

  void imu_odom_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    if (first_imu < 5) {
      imu_init_val = GetEulerZ(msg);
      q_first = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
      first_imu ++;
      return;
    }
    sensor_msgs::Imu modified_msg = *msg;
    modified_msg.header.stamp = ros::Time::now();
    modified_msg.header.frame_id = "d435_imu_optical_frame";

    Eigen::Quaterniond q_now(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
//  Eigen::Quaterniond q_diff = q_first.inverse() * q_now;
    Eigen::Quaterniond q_diff = Eigen::Quaterniond(Eigen::AngleAxisd(init_imu_yaw, Eigen::Vector3d::UnitZ())) * q_now;
    modified_msg.orientation.w = q_diff.w();
    modified_msg.orientation.x = q_diff.x();
    modified_msg.orientation.y = q_diff.y();
    modified_msg.orientation.z = q_diff.z();

    modified_msg.orientation_covariance = {imu_ori_cov, 0., 0.,
                                           0., imu_ori_cov, 0.,
                                           0., 0., imu_ori_cov};
    modified_msg.angular_velocity_covariance = {imu_ori_cov, 0., 0.,
                                                0., imu_ori_cov, 0.,
                                                0., 0., imu_ori_cov};
    modified_msg.linear_acceleration_covariance = {imu_ori_cov, 0., 0.,
                                                   0., imu_ori_cov, 0.,
                                                   0., 0., imu_ori_cov};

    imu_odom.publish(modified_msg);
  }

  ros::NodeHandle node;

  ros::Publisher vo_odom;
  ros::Publisher uwb_odom;
  ros::Publisher imu_odom;

  tf::TransformListener listener;

  bool first_vo = true;
  int first_uwb = 0;
  int first_imu = 0;
  double imu_init_val = 0;
  Eigen::Quaterniond q_first;

  double vo_pos_cov = 0.0;
  double vo_vel_cov = 0.0;
  double uwb_pos_cov = 0.0;
  double imu_ori_cov = 0.01;
  double init_odom_yaw = -0.75 * M_PI;
  double init_imu_yaw = -0.75 * M_PI;
  double distance_UWB_to_base = 0.508;

  Eigen::Matrix4d first_cam_odom = Eigen::Matrix4d::Identity();
  Eigen::Vector3d first_uwb_pos = Eigen::Vector3d::Zero();
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "TF_Odom");
  OdomFilter of;
  ros::Subscriber vo_odom_listener = of.node.subscribe("t265/odom/sample", 100, &OdomFilter::vo_odom_callback, &of);
  ros::Subscriber uwb_odom_listener = of.node.subscribe("dwm_odom", 100, &OdomFilter::uwb_odom_callback, &of);
  ros::Subscriber imu_listener = of.node.subscribe("rtabmap/imu", 100, &OdomFilter::imu_odom_callback, &of);
  ros::spin();
  return 0;
}
