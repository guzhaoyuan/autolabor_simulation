//
// Created by zhaoyuan on 3/14/20.
//

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "nav_msgs/Path.h"

class PathVisualization {
 public:
  PathVisualization () {
    ros::NodeHandle private_node("~");
    private_node.param("rate", rate_, 5);
    private_node.param("delay_start", delay_start, 1.0);
    ros::Duration(delay_start).sleep();
    path_pub_ = nh_.advertise<nav_msgs::Path>("base_link_path", 10);
    pub_path_timer_ = nh_.createTimer(ros::Duration(1.0/rate_), &PathVisualization::pubPathCallback, this);
  }

  void pubPathCallback(const ros::TimerEvent &event) {

    path_.header.frame_id = "map";
    path_.header.stamp = ros::Time::now();

    tf::StampedTransform odom_to_base_link;
    try {
      tf_listener_.lookupTransform("/map", "/base_link",
                               ros::Time(0), odom_to_base_link);
    } catch (tf::TransformException ex) {
      ROS_WARN("Cannot get tf: base_link in map frame");
      return;
    }
    geometry_msgs::PoseStamped p;
    p.pose.position.x = odom_to_base_link.getOrigin().getX();
    p.pose.position.y = odom_to_base_link.getOrigin().getY();

    path_.poses.push_back(p);
    path_pub_.publish(path_);
  }

  ros::NodeHandle nh_;
  tf::TransformListener tf_listener_;
  ros::Publisher path_pub_;
  ros::Timer pub_path_timer_;
  int rate_;
  nav_msgs::Path path_;
  double delay_start;
};

int main (int argc, char **argv) {
  ros::init(argc, argv, "simulation_base_node");
  PathVisualization pv;
  ros::spin();
  return 0;
}