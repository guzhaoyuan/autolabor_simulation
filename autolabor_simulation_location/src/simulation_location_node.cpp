

#include <autolabor_simulation_location/simulation_location_node.h>


namespace autolabor_simulation {

    SimulationLocation::SimulationLocation() {
        ros::NodeHandle private_node("~");
        private_node.param<std::string>("baselink_frame", baselink_frame_, "base_link");
        private_node.param<std::string>("real_map_frame", real_map_frame_, "real_map");
        private_node.param<std::string>("location_frame", location_frame_, "map");
        private_node.param<int>("rate", rate_, 10);

        //double location_to_real_map_x_, location_to_real_map_y_, location_to_real_map_yaw_;
        private_node.param<double>("location_to_real_map_x", location_to_real_map_x_, 0.0);
        private_node.param<double>("location_to_real_map_y", location_to_real_map_y_, 0.0);
        private_node.param<double>("location_to_real_map_yaw", location_to_real_map_yaw_, 0.0);
        location_to_realmap_.setRotation(tf::createQuaternionFromYaw(location_to_real_map_yaw_));
        location_to_realmap_.setOrigin(tf::Vector3(location_to_real_map_x_, location_to_real_map_y_, 0));
    }

    void SimulationLocation::pubLocationCallback(const ros::TimerEvent &event) {
        tf::StampedTransform realmap_to_baselink;
        if (tf_.canTransform(real_map_frame_, baselink_frame_, ros::Time())) {
            tf_.lookupTransform(real_map_frame_, baselink_frame_, ros::Time(), realmap_to_baselink);

            tf::Transform location_to_baselink = location_to_realmap_ * realmap_to_baselink;

            geometry_msgs::PointStamped location_to_baselink_msg;
            location_to_baselink_msg.header.stamp = ros::Time::now();
            location_to_baselink_msg.header.frame_id = location_frame_;

            location_to_baselink_msg.point.x = location_to_baselink.getOrigin().x();
            location_to_baselink_msg.point.y = location_to_baselink.getOrigin().y();
            location_to_baselink_msg.point.z = location_to_baselink.getOrigin().z();

            location_pub_.publish(location_to_baselink_msg);
        }
    }

    void SimulationLocation::pubEstimationCallback(const ros::TimerEvent &event) {
        tf::StampedTransform base_link_to_map_transform;
        try {
            // Get the latest transform.
            tf_.lookupTransform("map", "base_link", ros::Time(0), base_link_to_map_transform);
            geometry_msgs::TransformStamped real_map_to_robot2_trans;
            tf::transformStampedTFToMsg(base_link_to_map_transform, real_map_to_robot2_trans);
            real_map_to_robot2_trans.header.frame_id = real_map_frame_;
            real_map_to_robot2_trans.child_frame_id = "robot2/base_link";
            tf_broadcaster_.sendTransform(real_map_to_robot2_trans);
        } catch (tf2::TransformException &ex) {
            // ROS_INFO("tf2_ros::Buffer::lookupTransform failed: %s", ex.what());
        }
    }

    void SimulationLocation::run() {
        location_pub_ = nh_.advertise<geometry_msgs::PointStamped>("location_pos", 100);
        pub_location_timer_ = nh_.createTimer(ros::Duration(1.0 / rate_), &SimulationLocation::pubEstimationCallback, this);
        ros::spin();
    }

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "simulation_location_node");
    autolabor_simulation::SimulationLocation simulation_location;
    simulation_location.run();
    return 0;
}