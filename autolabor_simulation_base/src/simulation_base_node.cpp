#include <math.h>

#include "vector"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "autolabor_simulation_base/simulation_base_node.h"

namespace autolabor_simulation {

SimulationBase::SimulationBase():cur_v_linear_(0.0),cur_v_theta_(0.0),tar_v_linear_(0.0),tar_v_theta_(0.0),
  real_x_(0.0),real_y_(0.0),real_th_(0.0),odom_x_(0.0),odom_y_(0.0),odom_th_(0.0){
  ros::NodeHandle private_node("~");
  private_node.param("odom_frame", odom_frame_, std::string("odom"));
  private_node.param("base_link_frame", base_link_frame_, std::string("base_link"));
  private_node.param("real_map_frame", real_map_frame_, std::string("real_map"));
  private_node.param("map_frame", map_frame_, std::string("map"));

  private_node.param("noise_v_linear", noise_v_linear_, 0.0);
  private_node.param("noise_v_theta", noise_v_theta_, 0.0);

  private_node.param("max_a_linear", max_a_linear_, 999.0);
  private_node.param("max_a_theta", max_a_theta_, 999.0);

  private_node.param("max_v_linear", max_v_linear_, 1.0);
  private_node.param("max_v_theta", max_v_theta_, 1.57);

  private_node.param("rate", rate_, 30);
  private_node.param("rate_positioning", rate_pos_, 10);

  private_node.param("is_tf_broadcast", is_tf_broadcast_, true);

  last_time_ = ros::Time::now();
}

double SimulationBase::gaussRand(double mu, double sigma){
  static double V1, V2, S;
  static int phase = 0;
  double X;

  if ( phase == 0 ) {
    do {
      double U1 = (double)rand() / RAND_MAX;
      double U2 = (double)rand() / RAND_MAX;

      V1 = 2 * U1 - 1;
      V2 = 2 * U2 - 1;
      S = V1 * V1 + V2 * V2;
    } while(S >= 1 || S == 0);

    X = V1 * sqrt(-2 * log(S) / S);
  } else
    X = V2 * sqrt(-2 * log(S) / S);

  phase = 1 - phase;
  return (X*sigma + mu);
}

void SimulationBase::cmdReceived(const geometry_msgs::Twist::ConstPtr& cmd){
  cmd_mutex_.lock();
  tar_v_linear_ = cmd->linear.x;
  tar_v_theta_ = cmd->angular.z;
  cmd_mutex_.unlock();
}

void SimulationBase::updateOdometry(){
  double dt = (current_time_ - last_time_).toSec();
  
  if (tar_v_linear_ > cur_v_linear_){
    cur_v_linear_ = std::min(std::min(tar_v_linear_, max_v_linear_), cur_v_linear_ + max_a_linear_*dt);
  }else if (tar_v_linear_ < cur_v_linear_){
    cur_v_linear_ = std::max(std::max(-max_v_linear_, tar_v_linear_), cur_v_linear_ - max_a_linear_*dt);
  }else {
    cur_v_linear_ = tar_v_linear_;
  }
  ds = cur_v_linear_ * dt;
  ns = ds == 0 ? 0 : (cur_v_linear_ + gaussRand(0, noise_v_linear_)) * dt;

  if (tar_v_theta_ > cur_v_theta_){
    cur_v_theta_ = std::min(std::min(tar_v_theta_, max_v_theta_), cur_v_theta_ + max_a_theta_*dt);
  }else if (tar_v_theta_ < cur_v_theta_){
    cur_v_theta_ = std::max(std::max(-max_v_theta_, tar_v_theta_), cur_v_theta_ - max_a_theta_*dt);
  }else{
    cur_v_theta_ = tar_v_theta_;
  }
  dth = cur_v_theta_ * dt;
  nth = dth == 0 ? 0 : (cur_v_theta_ + gaussRand(0, noise_v_theta_)) * dt;

  real_th_ += dth;
  real_x_ += ds * cos(real_th_);
  real_y_ += ds * sin(real_th_);

  odom_th_ += nth;
  odom_x_ += ns * cos(odom_th_);
  odom_y_ += ns * sin(odom_th_);
}

void SimulationBase::pubOdomCallback(const ros::TimerEvent &event){
  current_time_ = ros::Time::now();
  updateOdometry();
  
  // tf : odom --> base_link
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time_;
  odom_trans.header.frame_id = odom_frame_;
  odom_trans.child_frame_id = base_link_frame_;
  odom_trans.transform.translation.x = odom_x_;
  odom_trans.transform.translation.y = odom_y_;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(odom_th_);
  if (is_tf_broadcast_) {
    tf_broadcaster_.sendTransform(odom_trans);
  }

  geometry_msgs::TransformStamped real_map_trans;
  real_map_trans.header.stamp = current_time_;
  real_map_trans.header.frame_id = base_link_frame_;
  real_map_trans.child_frame_id = real_map_frame_;
  tf::Transform realmap2baselink(tf::createQuaternionFromYaw(real_th_), tf::Vector3(real_x_, real_y_, 0));
  tf::Transform baselink2realmap = realmap2baselink.inverse();
  real_map_trans.transform.translation.x = baselink2realmap.getOrigin().getX();
  real_map_trans.transform.translation.y = baselink2realmap.getOrigin().getY();
  real_map_trans.transform.translation.z = 0.0;
  real_map_trans.transform.rotation = tf::createQuaternionMsgFromYaw(tf::getYaw(baselink2realmap.getRotation()));
  // assert(real_map_trans.transform.translation.x - (-cos(real_th_)*real_x_-sin(real_th_)*real_y_)<1e-10);
  // assert(real_map_trans.transform.translation.y - (sin(real_th_)*real_x_-cos(real_th_)*real_y_)<1e-10);
  // assert(tf::getYaw(baselink2realmap.getRotation()) - (-real_th_) < 1e-10);
  tf_broadcaster_.sendTransform(real_map_trans);

  // publish odometry topic
  nav_msgs::Odometry odom_msg;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_link_frame_;
  odom_msg.header.stamp = current_time_;
  odom_msg.pose.pose.position.x = odom_x_;
  odom_msg.pose.pose.position.y = odom_y_;
  odom_msg.pose.pose.position.z = 0;
  odom_msg.pose.pose.orientation.x = odom_trans.transform.rotation.x;
  odom_msg.pose.pose.orientation.y = odom_trans.transform.rotation.y;
  odom_msg.pose.pose.orientation.z = odom_trans.transform.rotation.z;
  odom_msg.pose.pose.orientation.w = odom_trans.transform.rotation.w;
  // odometry pose covariance.
  odom_msg.pose.covariance = boost::array<double, 36>({
    std::pow(0 + noise_v_linear_, 2) * 1, 0., 0., 0., 0., 0.,
    0., std::pow(0 + noise_v_linear_, 2)  * 1, 0., 0., 0., 0.,
    0., 0., 0., 0., 0., 0.,
    0., 0., 0., 0., 0., 0.,
    0., 0., 0., 0., 0., 0.,
    0., 0., 0., 0., 0., std::pow(0 + noise_v_theta_, 2) * 1});

  odom_msg.twist.twist.linear.x = ns;
  odom_msg.twist.twist.angular.z = nth;
  // odometry twist covariance.
  odom_msg.twist.covariance = boost::array<double, 36>({
    std::pow(0 + noise_v_linear_, 2), 0., 0., 0., 0., 0.,
    0., 0., 0., 0., 0., 0.,
    0., 0., 0., 0., 0., 0.,
    0., 0., 0., 0., 0., 0.,
    0., 0., 0., 0., 0., 0.,
    0., 0., 0., 0., 0., std::pow(cur_v_linear_ * (0 + noise_v_theta_), 2)});
  odom_pub_.publish(odom_msg);

  geometry_msgs::TwistWithCovarianceStamped twist;
  twist.header = odom_msg.header;
  twist.header.frame_id = base_link_frame_;
  twist.twist = odom_msg.twist;
  twist_pub_.publish(twist); // publish twist topic.

  // update time
  last_time_ = current_time_;
}

void SimulationBase::pubPosCallback(const ros::TimerEvent &event){
  // Distort real pose to get a 'measurement'.
  geometry_msgs::PoseWithCovarianceStamped beacon_data; // map -> base_link.
  beacon_data.header.frame_id = map_frame_;
  beacon_data.header.stamp = ros::Time::now();
  const double noise_x_theta_ = 0.2, noise_x_linear_ = 0.2;
  beacon_data.pose.pose.position.x = real_x_ + gaussRand(0, noise_x_linear_);
  beacon_data.pose.pose.position.y = real_y_ + gaussRand(0, noise_x_linear_);
  auto beacon_q = tf::createQuaternionMsgFromYaw(real_th_ + gaussRand(0, noise_x_theta_));
  beacon_data.pose.pose.orientation.x = beacon_q.x;
  beacon_data.pose.pose.orientation.y = beacon_q.y;
  beacon_data.pose.pose.orientation.z = beacon_q.z;
  beacon_data.pose.pose.orientation.w = beacon_q.w;
  // Tune here to specify how trusty the positioning system is.
  beacon_data.pose.covariance = boost::array<double, 36>({
    std::pow(0 + noise_x_linear_, 2) * 1, 0., 0., 0., 0., 0.,
    0., std::pow(0 + noise_x_linear_, 2) * 1, 0., 0., 0., 0.,
    0., 0., 0., 0., 0., 0.,
    0., 0., 0., 0., 0., 0.,
    0., 0., 0., 0., 0., 0.,
    0., 0., 0., 0., 0., std::pow(0 + noise_x_theta_, 2) * 1 });

  pos_pub_.publish(beacon_data);
}

void SimulationBase::run(){
  cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &SimulationBase::cmdReceived, this);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);
  twist_pub_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("twist", 10);
  pos_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 10);
  pub_odom_timer_ = nh_.createTimer(ros::Duration(1.0/rate_), &SimulationBase::pubOdomCallback, this);
  pub_pos_timer_ = nh_.createTimer(ros::Duration(1.0/rate_pos_), &SimulationBase::pubPosCallback, this);
  ros::spin();
  // while (ros::ok()) {
  //   ros::spinOnce();
  //   std::cout << rate_ << std::endl;
  // }
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simulation_base_node");
  autolabor_simulation::SimulationBase simulation_base;
  simulation_base.run();
  return 0;
}
