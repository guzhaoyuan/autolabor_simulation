//
// Created by zhaoyuan on 3/14/20.
//

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "nav_msgs/Path.h"
#include <xlnt/xlnt.hpp>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>

class PathVisualization {
 public:
  PathVisualization () {
    ros::NodeHandle private_node("~");
    private_node.param("rate", rate_, 5);
    private_node.param("delay_start", delay_start, 1.0);
    private_node.param<std::string>("target_frame", target_frame,
                                    "map");
    ros::Duration(delay_start).sleep();
    path_pub_ = nh_.advertise<nav_msgs::Path>("base_link_path", 10);
    pub_path_timer_ = nh_.createTimer(ros::Duration(1.0/rate_),
        &PathVisualization::pubPathCallback, this);
  }

  void pubPathCallback(const ros::TimerEvent &event) {

    path_.header.frame_id = target_frame;
    path_.header.stamp = ros::Time::now();

    tf::StampedTransform odom_to_base_link;
    try {
      tf_listener_.lookupTransform(target_frame, "base_link",
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
  std::string target_frame;
};

class GroundTruthVisualization {
 public:
  GroundTruthVisualization () {
    ros::NodeHandle private_node("~");
    private_node.param("rate", rate_, 1);
    private_node.param("delay_start", delay_start, 1.0);
    private_node.param<std::string>("begin_point", begin_point,
        "TPS_Auto_0202420");
    private_node.param<std::string>("end_point", end_point,
        "TPS_Auto_0203226");

    ros::Duration(delay_start).sleep();

    ground_truth_pub_ = nh_.advertise<nav_msgs::Path>("ground_truth", 10);
    ground_truth_flat_pub_ = nh_.advertise<nav_msgs::Path>("ground_truth_flat", 10);
    pub_path_timer_ = nh_.createTimer(ros::Duration(1.0/rate_),
        &GroundTruthVisualization::pubPathCallback, this);

    try {
      wb.load("/home/gzy/nav_ws/src/autolabor_simulation/imr/data/processed_data.xlsx");
      PathLoader();
    } catch (const std::exception& e) {
      ROS_ERROR(e.what());
    }
  }

  void pubPathCallback(const ros::TimerEvent &event) {
    ground_truth_pub_.publish(path_);
    ground_truth_flat_pub_.publish(flat_path_);
  }

  void PathLoader() {
    xlnt::worksheet data_sheet = wb.sheet_by_title("Test Points");
    const int num_points = data_sheet.rows(false).length();
    Eigen::Matrix<double, 3, Eigen::Dynamic> points(3, num_points);
    std::vector<std::string> data;
    for (auto row : data_sheet.rows(false))
    {
      for (auto cell : row)
      {
        data.push_back(cell.to_string());
//        std::cout << data.back() << std::endl;
      }
    }

    // Load data and find data range.
    for (int i = 0; i < data.size(); i++) {
      if (i%4 == 0) {
        if (data[i] == begin_point)
          begin_idx = i/4;
        if (data[i] == end_point)
          end_idx = i/4;
      } else
        points(i%4-1, i/4) = std::stod(data[i]);
    }

    // Load points into ROS path message.
    if (begin_idx != 0 && end_idx != 0 && begin_idx > end_idx) {
      Eigen::Matrix<double, 3, Eigen::Dynamic> path_points =
          points.block(0, end_idx, 3, begin_idx - end_idx);

      path_.header.stamp = ros::Time::now();
      flat_path_.header.stamp = ros::Time::now();
      path_.header.frame_id = "map";
      flat_path_.header.frame_id = "map";
      for (int j = 0; j < path_points.cols(); j++) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = path_points(0,j);
        pose_stamped.pose.position.y = path_points(1,j);
        pose_stamped.pose.position.z = path_points(2,j);
        path_.poses.push_back(pose_stamped);
        pose_stamped.pose.position.z = 0;
        flat_path_.poses.push_back(pose_stamped);
      }
      ROS_INFO("Ground truth processing complete.");
    } else {
      throw std::runtime_error("Did not find matching points range.");
    }
  }

  ros::NodeHandle nh_;

  ros::Publisher ground_truth_pub_;
  ros::Publisher ground_truth_flat_pub_;
  ros::Timer pub_path_timer_;

  xlnt::workbook wb;
  std::string begin_point;
  std::string end_point;
  int begin_idx = 0;
  int end_idx = 0;

  nav_msgs::Path path_;
  nav_msgs::Path flat_path_;

  int rate_;
  double delay_start;
};

class VOVisualization {
 public:
  VOVisualization () {
    ros::NodeHandle private_node("~");
    private_node.param("rate", rate_, 1);
    private_node.param("delay_start", delay_start, 1.0);
    private_node.param<std::string>("relative_frame", relative_frame,
                                    "odom");
    ros::Duration(delay_start).sleep();
    path_pub_ = nh_.advertise<nav_msgs::Path>("vo_path", 10);\
    pub_path_timer_ = nh_.createTimer(ros::Duration(1.0/rate_),
                                      &VOVisualization::pubPathCallback, this);
    vo_odom_listener_ = nh_.subscribe("t265/odom/sample",
                                      100, &VOVisualization::vo_odom_callback, this);
  }

  void pubPathCallback(const ros::TimerEvent &event) {
    path_pub_.publish(path_);
  }

  void vo_odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    path_.header.frame_id = relative_frame;
    path_.header.stamp = ros::Time::now();

    geometry_msgs::PoseStamped p;
    p.pose.position.x = msg->pose.pose.position.x;
    p.pose.position.y = msg->pose.pose.position.y;

    path_.poses.push_back(p);
  }

  ros::NodeHandle nh_;
  ros::Publisher path_pub_;
  ros::Timer pub_path_timer_;
  ros::Subscriber vo_odom_listener_;
  int rate_;
  nav_msgs::Path path_;
  double delay_start;
  std::string relative_frame;
};

int main (int argc, char **argv) {
  ros::init(argc, argv, "path_visualization_node");

  PathVisualization pv;
  GroundTruthVisualization gtv;
  VOVisualization vov;

  ros::spin();
  return 0;
}