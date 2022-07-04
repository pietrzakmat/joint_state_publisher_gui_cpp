#include "qrosnode.h"

QRosNode::QRosNode(int argc, char** argv, QObject *parent) : QObject(parent)
{
    // ROS2
    rclcpp::init(argc, argv);
    m_ros_node = rclcpp::Node::make_shared("joint_state_publisher_gui_cpp_node");

    auto ros_executor = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
    ros_executor->add_node(m_ros_node);
    std::thread executor_thread(std::bind(&rclcpp::executors::StaticSingleThreadedExecutor::spin, ros_executor));
    executor_thread.detach();

    // create publishers :
    joint_pub = m_ros_node->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SystemDefaultsQoS());
    // assign subscriptions
    m_robot_descriptor_sub = m_ros_node->create_subscription<std_msgs::msg::String>("robot_description",
                                                                                   m_qos,
                                                                                   std::bind(&QRosNode::m_robot_descriptor_callback, this, std::placeholders::_1));
}


void QRosNode::m_robot_descriptor_callback(const std_msgs::msg::String::SharedPtr msg)
{
    m_robot_descriptor_str = msg->data;
    Q_EMIT msgSubscribed();
}
