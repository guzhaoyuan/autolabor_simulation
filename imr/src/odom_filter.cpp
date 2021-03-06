#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf_conversions/tf_eigen.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace rvt = rviz_visual_tools;

class OdomFilter {
 public:
  OdomFilter() {

    ros::NodeHandle private_node("~");

    private_node.getParam("tag_pos_cov", tag_pos_cov);
    private_node.getParam("tag_ori_cov", tag_ori_cov);
    private_node.getParam("vo_pos_cov", vo_pos_cov);
    private_node.getParam("vo_ori_cov", vo_ori_cov);
    private_node.getParam("vo_vel_cov", vo_vel_cov);
    private_node.getParam("uwb_pos_cov", uwb_pos_cov);
    private_node.getParam("uwb_ori_cov", uwb_ori_cov);
    private_node.getParam("init_odom_yaw", init_odom_yaw);
    private_node.getParam("init_imu_yaw", init_imu_yaw);
    private_node.getParam("imu_ori_cov", imu_ori_cov);
    private_node.getParam("distance_UWB_to_base", distance_UWB_to_base);
    private_node.getParam("distance_UWB_to_base_X", distance_UWB_to_base_X);
    private_node.getParam("distance_UWB_to_base_Y", distance_UWB_to_base_Y);
    private_node.getParam("distance_camera_to_base", distance_camera_to_base);

    init_ros();
    init_visual_tool();
    ros::Duration(1).sleep();
    check_data();
  }

  void init_visual_tool() {

    visual_tools_.reset(new rvt::RvizVisualTools("map", "/rviz_visual_tools"));
    visual_tools_->loadMarkerPub();  // create publisher before waiting
    visual_tools_->deleteAllMarkers();

  }

  void init_ros() {
    vo_odom = node.advertise<nav_msgs::Odometry>("t265/odom", 10);
    uwb_odom = node.advertise<nav_msgs::Odometry>("dwm/odom", 10);
    global_uwb_odom = node.advertise<nav_msgs::Odometry>("dwm/global_odom", 10);
    imu_odom = node.advertise<sensor_msgs::Imu>("imu/odom", 10);
    global_imu_odom = node.advertise<sensor_msgs::Imu>("imu/global_odom", 10);
    tag_odom = node.advertise<nav_msgs::Odometry>("tag/odom", 10);

    vo_odom_listener = node.subscribe("t265/odom/sample",
        100, &OdomFilter::vo_odom_callback, this);
//    uwb_odom_listener = node.subscribe("dwm_odom", 100, &OdomFilter::uwb_odom_callback, this);
    luwb_pos_listener = node.subscribe("lhs/dwm1001/tag",
        10, &OdomFilter::uwb_lpos_callback, this);
    ruwb_pos_listener = node.subscribe("rhs/dwm1001/tag",
        10, &OdomFilter::uwb_rpos_callback, this);
    imu_listener = node.subscribe("magic/imu", 50,
        &OdomFilter::imu_odom_callback, this);
    fiducial_listener = node.subscribe("tag_detections", 100,
        &OdomFilter::fiducial_CB, this);

    pub_pos_timer_ = node.createTimer(ros::Duration(1.0/10),
        &OdomFilter::uwb_callback, this);

    button_listener = node.subscribe("rviz_visual_tools_gui", 100,
                                     &OdomFilter::GlobalDataControl, this);

// Failed to triger callback for unknown reason. Do not use Synchronizer for now
//    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs
//    ::PointStamped, geometry_msgs::PointStamped> MySyncPolicy;
//    MySyncPolicy my_sync_policy(100);
//    my_sync_policy.setInterMessageLowerBound(0, ros::Duration(1));
//    message_filters::Subscriber<geometry_msgs::PointStamped> luwb_sync(node,
//        "lhs/dwm1001/tag", 100);
//    message_filters::Subscriber<geometry_msgs::PointStamped> ruwb_sync(node,
//        "rhs/dwm1001/tag", 100);
//    message_filters::Synchronizer<MySyncPolicy> sync(static_cast<const MySyncPolicy &>(my_sync_policy),
//                                                     luwb_sync, ruwb_sync);
//    sync.registerCallback(boost::bind(&OdomFilter::uwb_callback, this, _1, _2));
  }

  void check_data() {

  }

  void GlobalDataControl(const sensor_msgs::Joy::ConstPtr& joy_msg) {
    send_global_data = !send_global_data;
    if (send_global_data)
      ROS_INFO("Start sending global data (Fiducial detection/IMU).");
    else
      ROS_INFO("Stop sending global data (Fiducial detection/IMU).");
  }

  // Visual Odometry goes into local EKF.
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
                                    0., 0., 0., 0., 0., vo_ori_cov};

    modified_msg.twist.covariance = {vo_vel_cov, 0., 0., 0., 0., 0.,
                                     0., vo_vel_cov, 0., 0., 0., 0.,
                                     0., 0., 0., 0., 0., 0.,
                                     0., 0., 0., 0., 0., 0.,
                                     0., 0., 0., 0., 0., 0.,
                                     0., 0., 0., 0., 0., vo_vel_cov};
    vo_odom.publish(modified_msg);
  }

  void uwb_lpos_callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    luwb_pos_O = Eigen::Vector3d(msg->point.x, msg->point.y-0.5, msg->point.z);
//    if (first_f) // Do nothing if fiducial not initialized.
//      return;
//
//    // Record the initial data report.
//    if (first_luwb < 5) {
//      first_luwb ++;
//      first_luwb_pos = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);
//      return;
//    }

//    { // Transform UWB in odom frame and send to EKF in local_msg.
//      nav_msgs::Odometry local_msg;
//      local_msg.header.stamp = ros::Time::now();
//      local_msg.header.frame_id = "luwb_odom";
//      local_msg.child_frame_id = "base_link";
//
//      // Try to rotate the UWB sensor reading to align with the map frame.
//      // Assume the first UWB reading is at the odom frame. To make alignment, the initial odom frame angle is guessed.
//      Eigen::Vector3d position_W = Eigen::Vector3d(msg->x, msg->y, msg->z) - first_luwb_pos;
//
//      Eigen::Vector3d position_UWB = Eigen::AngleAxisd(init_odom_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
//          position_W;
//
//      tf::StampedTransform odom_to_base_link;
//      try {
//        listener.lookupTransform("/odom", "/base_link",
//                                 ros::Time(0), odom_to_base_link);
//      } catch (tf::TransformException ex) {
//        odom_to_base_link.setIdentity();
//      }
//      tf::Quaternion q = odom_to_base_link.getRotation();
//      Eigen::Quaterniond q_OB(q.w(),q.x(),q.y(),q.z());
//      Eigen::Matrix3d R_OB = q_OB.toRotationMatrix();
//      // Compute the base_link in uwb_odom frame by adding offset from UWB.
//      Eigen::Vector3d position_base = position_UWB + R_OB.transpose() *
//          Eigen::Vector3d(distance_UWB_to_base_X, distance_UWB_to_base_Y, 0);
//      local_msg.pose.pose.position.x = position_base(0);
//      local_msg.pose.pose.position.y = position_base(1);
//      local_msg.pose.pose.position.z = 0;
//
//      local_msg.pose.covariance = {uwb_pos_cov, 0., 0., 0., 0., 0.,
//                                   0., uwb_pos_cov, 0., 0., 0., 0.,
//                                   0., 0., 0., 0., 0., 0.,
//                                   0., 0., 0., 0., 0., 0.,
//                                   0., 0., 0., 0., 0., 0.,
//                                   0., 0., 0., 0., 0., 0.};
//
//      uwb_odom.publish(local_msg);
//    }

//    { // Transform UWB in map frame and send to EKF in global_msg.
//      nav_msgs::Odometry global_msg;
//      global_msg.header.stamp = ros::Time::now();
//      global_msg.header.frame_id = "map";
//      global_msg.child_frame_id = "base_link";
//
//      Eigen::Vector3d position_UWB_W = Eigen::Vector3d(msg->x, msg->y +
//      distance_world_to_beacon_Y, 0);
//
//      tf::StampedTransform map_to_base_link;
//      try {
//        listener.lookupTransform("/map", "/base_link",
//                                 ros::Time(0), map_to_base_link);
//      } catch (tf::TransformException ex) {
//        return;
//      }
//
//      tf::Quaternion q = map_to_base_link.getRotation();
//      Eigen::Quaterniond q_WB(q.w(),q.x(),q.y(),q.z());
//      Eigen::Matrix3d R_WB = q_WB.toRotationMatrix();
//      // Compute the base_link in uwb_odom frame by adding offset from UWB.
//      Eigen::Vector3d position_base_W = position_UWB_W + R_WB.transpose() *
//          Eigen::Vector3d(distance_UWB_to_base_X, distance_UWB_to_base_Y, 0);
//
//      global_msg.pose.pose.position.x = position_base_W(0);
//      global_msg.pose.pose.position.y = position_base_W(1);
//      global_msg.pose.pose.position.z = 0;
//
//      global_msg.pose.covariance = {uwb_pos_cov, 0., 0., 0., 0., 0.,
//                                   0., uwb_pos_cov, 0., 0., 0., 0.,
//                                   0., 0., 0., 0., 0., 0.,
//                                   0., 0., 0., 0., 0., 0.,
//                                   0., 0., 10., 0., 0., 0.,
//                                   0., 0., 0., 0., 0., 0.};
//
//      global_uwb_odom.publish(global_msg);
//
//      Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
//      pose1.translation() = position_base_W;
//      visual_tools_->publishSphere(visual_tools_->convertPose(pose1),
//                                   rvt::colors::BLUE,
//                                   visual_tools_->getScale(rvt::scales::XXLARGE),
//                                   "Sphere");
//      visual_tools_->trigger();
//    }
  }

  void uwb_rpos_callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    ruwb_pos_O = Eigen::Vector3d(msg->point.x, msg->point.y-0.5, msg->point.z);
  }

  // UWB data goes to global EKF.
  void uwb_callback(const ros::TimerEvent &event) {
    if (!send_global_data)
      return;

    { // Transform UWB data to map frame and send to global EKF.
      nav_msgs::Odometry global_msg;
      global_msg.header.stamp = ros::Time::now();
      global_msg.header.frame_id = "map";
      global_msg.child_frame_id = "base_link";

      Eigen::Vector3d pos_l_W = Eigen::Vector3d(luwb_pos_O(0), luwb_pos_O(1) +
          distance_world_to_beacon_Y, 0);
      Eigen::Vector3d pos_r_W = Eigen::Vector3d(ruwb_pos_O(0), ruwb_pos_O(1) +
          distance_world_to_beacon_Y, 0);
      Eigen::Vector3d pos_base_W = (pos_l_W + pos_r_W)/2;
      Eigen::Vector3d l_to_r_W = (pos_r_W - pos_l_W)/2;

      global_msg.pose.pose.position.x = pos_base_W(0);
      global_msg.pose.pose.position.y = pos_base_W(1);
      global_msg.pose.pose.position.z = 0;

      double theta_l_to_r = std::atan2(l_to_r_W(1), l_to_r_W(0));
      Eigen::Quaterniond q_heading_W(
        Eigen::AngleAxisd(theta_l_to_r, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()));

      global_msg.pose.pose.orientation.w = q_heading_W.w();
      global_msg.pose.pose.orientation.x = q_heading_W.x();
      global_msg.pose.pose.orientation.y = q_heading_W.y();
      global_msg.pose.pose.orientation.z = q_heading_W.z();

      global_msg.pose.covariance = {uwb_pos_cov, 0., 0., 0., 0., 0.,
                                    0., uwb_pos_cov, 0., 0., 0., 0.,
                                    0., 0., 0., 0., 0., 0.,
                                    0., 0., 0., 0., 0., 0.,
                                    0., 0., 0., 0., 0., 0.,
                                    0., 0., 0., 0., 0., uwb_ori_cov};

      global_uwb_odom.publish(global_msg);

      // Visualize the position.
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.translation() = pos_l_W;
      visual_tools_->publishSphere(visual_tools_->convertPose(pose),
                                   rvt::colors::RED,
                                   visual_tools_->getScale(rvt::scales::XXLARGE),
                                   "Sphere");
      pose.translation() = pos_base_W;
      visual_tools_->publishSphere(visual_tools_->convertPose(pose),
                                   rvt::colors::ORANGE,
                                   visual_tools_->getScale(rvt::scales::XXLARGE),
                                   "Sphere");
      pose.translation() = pos_r_W;
      visual_tools_->publishSphere(visual_tools_->convertPose(pose),
                                   rvt::colors::YELLOW,
                                   visual_tools_->getScale(rvt::scales::XXLARGE),
                                   "Sphere");
      visual_tools_->trigger();
    }
  }

  // Deprecated.
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
    // Assume the first UWB reading is at the odom frame.
    // To make alignment, the initial odom frame angle is guessed.
    Eigen::Vector3d position_W = Eigen::Vector3d(msg->pose.pose.position.x,
        msg->pose.pose.position.y, msg->pose.pose.position.z) - first_uwb_pos;
    Eigen::Vector3d position_UWB =
        Eigen::AngleAxisd(init_odom_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
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
    ROS_INFO("The IMU euler angles are: %.2f, %.2f, %.2f", euler[0],euler[1],
        euler[2]);

    return euler[0];
  }

  // IMU should be used in global and local EKF.
  // For now, only local EKF uses IMU data.
  void imu_odom_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    // Throw away first several imu data.
    if (first_imu < 20) {
      // Remember initial yaw angle and compensate later.
      imu_init_val = GetEulerZ(msg);
      ROS_INFO("imu_init_val: %.2f.", imu_init_val);
      q_first = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
      first_imu ++;
      return;
    }

    // This IMU data represents the /d435_imu_optical_frame in the
    // North-East-Down frame. init_imu_yaw represents the NED to /map rotation.

    { // Local imu data: represent base_link orientation on odom frame.
      sensor_msgs::Imu modified_msg = *msg;
      modified_msg.header.stamp = ros::Time::now();
      // Meaning the frame that imu data represents. The parent frame is NED.
      modified_msg.header.frame_id = "d435_imu_optical_frame";

      Eigen::Quaterniond q_now(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
      // IMU offset is not applicable here. Because in local EKF, IMU data is
      // in relative mode (subtract the first imu data is done in EKF).
      modified_msg.orientation.w = q_now.w();
      modified_msg.orientation.x = q_now.x();
      modified_msg.orientation.y = q_now.y();
      modified_msg.orientation.z = q_now.z();

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

    { // Global imu data: represent base_link orientation on map frame.
      sensor_msgs::Imu global_imu_msg = *msg;
      global_imu_msg.header.stamp = ros::Time::now();
      global_imu_msg.header.frame_id = "d435_imu_optical_frame";

      Eigen::Quaterniond q_now(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
      // The q_diff represents no rotation when the imu data first came in.
      Eigen::Quaterniond q_diff = Eigen::Quaterniond(
          Eigen::AngleAxisd(imu_init_val, Eigen::Vector3d::UnitZ())) * q_now;
      Eigen::Quaterniond q_base_W = Eigen::Quaterniond(
          Eigen::AngleAxisd(init_imu_yaw, Eigen::Vector3d::UnitZ())) * q_diff;
      global_imu_msg.orientation.w = q_base_W.w();
      global_imu_msg.orientation.x = q_base_W.x();
      global_imu_msg.orientation.y = q_base_W.y();
      global_imu_msg.orientation.z = q_base_W.z();

      global_imu_msg.orientation_covariance = {imu_ori_cov, 0., 0.,
                                             0., imu_ori_cov, 0.,
                                             0., 0., imu_ori_cov};
      global_imu_msg.angular_velocity_covariance = {imu_ori_cov, 0., 0.,
                                                  0., imu_ori_cov, 0.,
                                                  0., 0., imu_ori_cov};
      global_imu_msg.linear_acceleration_covariance = {imu_ori_cov, 0., 0.,
                                                     0., imu_ori_cov, 0.,
                                                     0., 0., imu_ori_cov};
      global_imu_odom.publish(global_imu_msg);
    }
  }

  // Fiducial data is used to initialize the odom frame. Not used in any EKF
  // yet.
  void fiducial_CB(const apriltag_ros::AprilTagDetectionArray::ConstPtr& tag_array) {
    if (!send_global_data)
      return;

    if (tag_array->detections.empty())
      return;

    // Verify ID.
    if (tag_array->detections[0].id[0] != 0)
      return;

    // Get the first detection between camera C and fiducial F.
    tf::Transform tf_CF;
    tf::poseMsgToTF(tag_array->detections[0].pose.pose.pose, tf_CF);
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_CF,
        ros::Time::now(), "d435_correct", "detected_fiducial"));

    tf::StampedTransform tf_WF;
    tf::StampedTransform tf_BC;
    try {
      listener.lookupTransform("/map", "/fiducial0",
                               ros::Time(0), tf_WF);
      listener.lookupTransform("/base_link", "/d435_correct",
                               ros::Time(0), tf_BC);
    } catch (tf::TransformException ex) {
      return;
    }

    tf::Transform tf_BF = tf_BC * tf_CF;
    tf::Transform tf_WB = tf_WF * tf_BF.inverse();
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_WB,
        ros::Time::now(), "map", "detected_base"));
    if (first_f) {
      tf_broadcaster_.sendTransform(tf::StampedTransform(tf_WB,
          ros::Time::now(), "map", "odom"));
      double roll, pitch, yaw;
      tf_WB.getBasis().getRPY(roll, pitch, init_odom_yaw);
      ROS_INFO("[Fiducial]: Map to Odom initialized. Init yaw: %.1f.",
          init_odom_yaw);
      first_f = false;
      return;
    }

    double distance = tf_CF.getOrigin().length();
    if (distance > 6 || distance < 1)
      return;
    try {
//      tf::StampedTransform tf_OB;
//      listener.lookupTransform("/odom", "/base_link",
//                               ros::Time(0), tf_OB);
//
//      tf::Transform tf_WO = tf_WB * tf_OB.inverse();

      nav_msgs::Odometry tag_msg;
      tag_msg.header.stamp = ros::Time::now();
      tag_msg.header.frame_id = "map";
      tag_msg.child_frame_id = "base_link";
      tag_msg.pose.pose.position.x = tf_WB.getOrigin().getX();
      tag_msg.pose.pose.position.y = tf_WB.getOrigin().getY();
      tag_msg.pose.pose.position.z = 0;
      tf::Quaternion q_WB = tf_WB.getRotation();
      tag_msg.pose.pose.orientation.x = q_WB.x();
      tag_msg.pose.pose.orientation.y = q_WB.y();
      tag_msg.pose.pose.orientation.z = q_WB.z();
      tag_msg.pose.pose.orientation.w = q_WB.w();
      tag_msg.pose.covariance = {tag_pos_cov, 0., 0., 0., 0., 0.,
                                      0., tag_pos_cov, 0., 0., 0., 0.,
                                      0., 0., 0., 0., 0., 0.,
                                      0., 0., 0., 0., 0., 0.,
                                      0., 0., 0., 0., 0., 0.,
                                      0., 0., 0., 0., 0., tag_ori_cov};

      tag_odom.publish(tag_msg);
    } catch (tf::TransformException ex) {
      return;
    }
  }

  ros::NodeHandle node;

  ros::Publisher vo_odom;
  ros::Publisher uwb_odom;
  ros::Publisher global_uwb_odom;
  ros::Publisher imu_odom;
  ros::Publisher global_imu_odom;
  ros::Publisher tag_odom;
  ros::Subscriber vo_odom_listener;
  ros::Subscriber uwb_odom_listener;
  ros::Subscriber luwb_pos_listener;
  ros::Subscriber ruwb_pos_listener;
  ros::Subscriber imu_listener;
  ros::Subscriber fiducial_listener;
  ros::Subscriber button_listener;
  ros::Timer pub_pos_timer_;

  tf::TransformListener listener;
  tf::TransformBroadcaster tf_broadcaster_;

  bool first_vo = true;
  bool first_f = true;
  int first_uwb = 0;
  int first_luwb = 0;
  int first_ruwb = 0;
  int first_imu = 0;
  double imu_init_val = 0;
  Eigen::Quaterniond q_first;

  double tag_pos_cov = 0.0;
  double tag_ori_cov = 0.0;
  double vo_pos_cov = 0.0;
  double vo_ori_cov = 0.0;
  double vo_vel_cov = 0.0;
  double uwb_pos_cov = 0.0;
  double uwb_ori_cov = 0.0;
  double imu_ori_cov = 0.01;
  double init_odom_yaw = -1 * M_PI;
  double init_imu_yaw = -0.75 * M_PI;
  double distance_UWB_to_base = 0.308;
  double distance_UWB_to_base_X = -0.02;
  double distance_UWB_to_base_Y = 0.28;
  double distance_camera_to_base = -0.20;
  const double distance_world_to_beacon_Y = 2.44;
  bool send_global_data = true;

  Eigen::Matrix4d first_cam_odom = Eigen::Matrix4d::Identity();
  Eigen::Vector3d first_uwb_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d first_luwb_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d first_ruwb_pos = Eigen::Vector3d::Zero();
  Eigen::Affine3d fiducial0_W;
  Eigen::Vector3d luwb_pos_O;
  Eigen::Vector3d ruwb_pos_O;

  rvt::RvizVisualToolsPtr visual_tools_;
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "TF_Odom");
  OdomFilter of;

  ros::spin();
  return 0;
}
