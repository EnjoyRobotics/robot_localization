#include "robot_localization/odom_transform_node.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/utilities.hpp"
#include <chrono>

using namespace std::chrono_literals;


OdomTransform::OdomTransform() : Node("odom_transform")
{
    // get parameters
    this->declare_parameter<std::string>("odom_topic");
    this->declare_parameter<std::string>("pose_topic");
    this->declare_parameter<std::string>("target_frame");

    this->get_parameter("odom_topic", odom_topic);
    this->get_parameter("pose_topic", pose_topic);
    this->get_parameter("target_frame", target_frame);

    RCLCPP_INFO(this->get_logger(), "Projecting '%s' to frame '%s' on topic '%s'",
        odom_topic.c_str(), target_frame.c_str(), pose_topic.c_str());

    // initialize tf2
    buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener = std::make_shared<tf2_ros::TransformListener>(*buffer);

    // initialize odom subscriber
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 10,
        std::bind(&OdomTransform::odom_callback, this, std::placeholders::_1));

    // initialize pose publisher
    pose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic, 10);
}


void OdomTransform::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
{
    // convert Odometry to Pose
    geometry_msgs::msg::Pose pose_before, pose;
    pose_before.position.x = msg->pose.pose.position.x;
    pose_before.position.y = msg->pose.pose.position.y;
    pose_before.position.z = msg->pose.pose.position.z;
    pose_before.orientation.x = msg->pose.pose.orientation.x;
    pose_before.orientation.y = msg->pose.pose.orientation.y;
    pose_before.orientation.z = msg->pose.pose.orientation.z;
    pose_before.orientation.w = msg->pose.pose.orientation.w;

    pose = buffer->transform(pose_before, target_frame);

    // create PoseWithCovarianceStamped
    geometry_msgs::msg::PoseWithCovariance posec;
    posec.pose = pose;
    posec.covariance = msg->pose.covariance;
    geometry_msgs::msg::PoseWithCovarianceStamped posecs;
    posecs.header = msg->header;
    posecs.header.frame_id = target_frame;
    posecs.pose = posec;

    // publish pose
    pose_pub->publish(posecs);
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomTransform>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
