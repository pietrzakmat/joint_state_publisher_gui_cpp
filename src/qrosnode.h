#ifndef QROSNODE_H
#define QROSNODE_H

#include <iostream>
#include <thread>
#include <memory>

#include <QObject>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class QRosNode : public QObject {
    Q_OBJECT
public:
    QRosNode(int argc = 0, char** argv = nullptr, QObject *parent = nullptr);
    rclcpp::Node::SharedPtr m_ros_node {nullptr};

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub;

    std::string m_robot_descriptor_str;

Q_SIGNALS:
    void msgSubscribed();

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_robot_descriptor_sub;

    void m_robot_descriptor_callback(const std_msgs::msg::String::SharedPtr msg);

    rclcpp::QoS m_qos = rclcpp::QoS(10)
                                    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                    .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

};

#endif // QROSNODE_H
