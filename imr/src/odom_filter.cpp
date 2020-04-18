#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

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
    private_node.getParam("distance_camera_to_base", distance_camera_to_base);

//    init();
    run();
  }

  void init() {
    tf::StampedTransform map_to_odom_link;
    while (1) { // TODO: This is a bad style, change this later.
      try {
        listener.lookupTransform("/odom", "/map",
                                 ros::Time(0), map_to_odom_link);
        break;
      } catch (tf::TransformException ex) {
        // Only start odometry if global orientation is detected.
        ros::Duration(1).sleep();
      }
    }
  }

  void run() {
    vo_odom = node.advertise<nav_msgs::Odometry>("t265/odom", 10);
    uwb_odom = node.advertise<nav_msgs::Odometry>("dwm/odom", 10);
    imu_odom = node.advertise<sensor_msgs::Imu>("imu/odom", 10);

    vo_odom_listener = node.subscribe("t265/odom/sample",
        100, &OdomFilter::vo_odom_callback, this);
    uwb_odom_listener = node.subscribe("dwm_odom", 100, &OdomFilter::uwb_odom_callback, this);
    luwb_pos_listener = node.subscribe("/lhs/dwm1001/tag",
                                                          10, &OdomFilter::uwb_lpos_callback, this);
    ruwb_pos_listener = node.subscribe("/rhs/dwm1001/tag",
                                                          10, &OdomFilter::uwb_rpos_callback, this);
    imu_listener = node.subscribe("rtabmap/imu", 100, &OdomFilter::imu_odom_callback, this);
    fiducial_listener = node.subscribe("tag_detections", 10,
        &OdomFilter::fiducial_CB, this);

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
    // Create transform from camera to base in current base_link frame.
    Eigen::Matrix4d T_cam_base = Eigen::Matrix4d::Identity();
    T_cam_base(0,3) = distance_camera_to_base;
    Eigen::Matrix4d curr_wrt_first = first_cam_odom.inverse() * curr_cam_odom * T_cam_base;

    tf::StampedTransform odom_to_base_link;
    try{
      listener.lookupTransform("/base_link", "/odom",
                               ros::Time(0), odom_to_base_link);
    } catch (tf::TransformException ex) {
      odom_to_base_link.setIdentity();
    }
    tf::Quaternion q = odom_to_base_link.getRotation();
    Eigen::Quaterniond q_WO(q.w(),q.x(),q.y(),q.z());
    Eigen::Matrix3d R_WO = q_WO.toRotationMatrix();
    // Compute the base_link in t265_odom_frame by adding offset from camera.
    Eigen::Vector3d position_base = curr_cam_odom.topRightCorner<3,1>() + R_WO.transpose() * Eigen::Vector3d
        (distance_camera_to_base, 0, 0);

    Eigen::Matrix3d curr_wrt_first_R = curr_wrt_first.topLeftCorner(3, 3);
    // Eigen::Matrix3d init_guess_for_odom_frame_yaw = Eigen::AngleAxisd(init_odom_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix3d init_guess_for_odom_frame_yaw = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Quaterniond curr_wrt_first_q(curr_wrt_first_R * init_guess_for_odom_frame_yaw);

    nav_msgs::Odometry modified_msg = *msg;
    modified_msg.header.stamp = ros::Time::now();
    modified_msg.header.frame_id = "t265_odom_frame";
    modified_msg.child_frame_id = "base_link";

//    // Transform from camera to base_link use angle from camera odom.
//    modified_msg.pose.pose.position.x = curr_wrt_first(0,3);
//    modified_msg.pose.pose.position.y = curr_wrt_first(1,3);
    // Transform from camera to base_link use angle from EKF estimate.
    modified_msg.pose.pose.position.x = position_base(0);
    modified_msg.pose.pose.position.y = position_base(1);
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

  void uwb_lpos_callback(const geometry_msgs::Point::ConstPtr& msg) {
    if (first_luwb < 5) {
      first_luwb ++;
      first_luwb_pos = Eigen::Vector3d(msg->x, msg->y, msg->z);
      return;
    }

    nav_msgs::Odometry modified_msg;
    modified_msg.header.stamp = ros::Time::now();
    modified_msg.header.frame_id = "uwb_odom";
    modified_msg.child_frame_id = "base_link";

    // Try to rotate the UWB sensor reading to align with the map frame.
    // Assume the first UWB reading is at the odom frame. To make alignment, the initial odom frame angle is guessed.
    Eigen::Vector3d position_W = Eigen::Vector3d(msg->x, msg->y, msg->z) - first_uwb_pos;
    Eigen::Vector3d position_UWB = Eigen::AngleAxisd(init_odom_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
        position_W;

    tf::StampedTransform odom_to_base_link;
    try {
      listener.lookupTransform("/base_link", "/odom",
                               ros::Time(0), odom_to_base_link);
    } catch (tf::TransformException ex) {
      odom_to_base_link.setIdentity();
    }
    tf::Quaternion q = odom_to_base_link.getRotation();
    Eigen::Quaterniond q_WO(q.w(),q.x(),q.y(),q.z());
    Eigen::Matrix3d R_WO = q_WO.toRotationMatrix();
    // Compute the base_link in uwb_odom frame by adding offset from UWB.
    Eigen::Vector3d position_base = position_UWB + R_WO.transpose() *
        Eigen::Vector3d(distance_UWB_to_base_X, distance_UWB_to_base_Y, 0);

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

  void uwb_rpos_callback(const geometry_msgs::Point::ConstPtr& msg) {}

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
    } catch (tf::TransformException ex) {
      odom_to_base_link.setIdentity();
    }
    tf::Quaternion q = odom_to_base_link.getRotation();
    Eigen::Quaterniond q_WO(q.w(),q.x(),q.y(),q.z());
    Eigen::Matrix3d R_WO = q_WO.toRotationMatrix();
    // Compute the base_link in uwb_odom frame by adding offset from UWB.
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
      // Throw away first several imu data.
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

  void fiducial_CB(const apriltag_ros::AprilTagDetectionArray::ConstPtr& tag_array) {
    if (tag_array->detections.empty())
      return;

    // Verify ID.
    if (tag_array->detections[0].id[0] != 0)
      return;

    // Get the first detection between camera C and fiducial F.
    tf::Transform tf_transform_CF;
    tf::poseMsgToTF(tag_array->detections[0].pose.pose.pose, tf_transform_CF);

    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_transform_CF,
        ros::Time::now(), "t265_fisheye1_optical_frame", "fiducial"));
  }

  ros::NodeHandle node;

  ros::Publisher vo_odom;
  ros::Publisher uwb_odom;
  ros::Publisher imu_odom;
  ros::Subscriber vo_odom_listener;
  ros::Subscriber uwb_odom_listener;
  ros::Subscriber luwb_pos_listener;
  ros::Subscriber ruwb_pos_listener;
  ros::Subscriber imu_listener;
  ros::Subscriber fiducial_listener;

  tf::TransformListener listener;
  tf::TransformBroadcaster tf_broadcaster_;

  bool first_vo = true;
  int first_uwb = 0;
  int first_luwb = 0;
  int first_ruwb = 0;
  int first_imu = 0;
  double imu_init_val = 0;
  Eigen::Quaterniond q_first;

  double vo_pos_cov = 0.0;
  double vo_vel_cov = 0.0;
  double uwb_pos_cov = 0.0;
  double imu_ori_cov = 0.01;
  double init_odom_yaw = -0.75 * M_PI;
  double init_imu_yaw = -0.75 * M_PI;
  double distance_UWB_to_base = 0.308;
  double distance_UWB_to_base_X = -0.02;
  double distance_UWB_to_base_Y = 0.28;
  double distance_camera_to_base = -0.20;

  Eigen::Matrix4d first_cam_odom = Eigen::Matrix4d::Identity();
  Eigen::Vector3d first_uwb_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d first_luwb_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d first_ruwb_pos = Eigen::Vector3d::Zero();
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "TF_Odom");
  OdomFilter of;

  ros::spin();
  return 0;
}
