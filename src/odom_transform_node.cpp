#include "robot_localization/odom_transform_node.hpp"
#include "rclcpp/logging.hpp"


OdomTransform::OdomTransform() : Node("odom_transform")
{
    // get parameters
    this->declare_parameter<std::string>("odom_topic");
    this->declare_parameter<std::string>("pose_topic");
    this->declare_parameter<std::string>("original_frame");
    this->declare_parameter<std::string>("new_frame");

    this->get_parameter("odom_topic", odom_topic);
    this->get_parameter("pose_topic", pose_topic);
    this->get_parameter("original_frame", original_frame);
    this->get_parameter("new_frame", new_frame);

    RCLCPP_INFO(this->get_logger(), "Projecting '%s' from '%s' to '%s' on topic '%s'",
        odom_topic.c_str(), original_frame.c_str(), new_frame.c_str(), pose_topic.c_str());

    // initialize tf2
    buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
    transform = buffer->lookupTransform(new_frame, original_frame, rclcpp::Time(0));

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

    // geometry_msgs::msg::Point pt1, pt2;
    // tf2::doTransform(pt1, pt2, transform);

    // create PoseWithCovarianceStamped
    geometry_msgs::msg::PoseWithCovariance posec;
    posec.pose = pose;
    posec.covariance = msg->pose.covariance;
    geometry_msgs::msg::PoseWithCovarianceStamped posecs;
    posecs.header = msg->header;
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
