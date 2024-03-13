#ifndef RM_SENTRY_2024_NAV_HEAL_NODE_
#define RM_SENTRY_2024_NAV_HEAL_NODE_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/point.hpp"
#include "robot_msgs/msg/robot_info.hpp"
#include "toml.hpp"
#include <utility>

namespace BehaviorTree{
    class nav_heal_node:public BT::SyncActionNode{
        public:
        rclcpp::Node::SharedPtr node1;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_heal_point;
        nav_heal_node(const std::string&name, const BT::NodeConfig& config);
        BT::NodeStatus tick() override;
        geometry_msgs::msg::Point heal_position;
        bool check_blood();
        static BT::PortsList providedPorts()
        {
            return{
                BT::InputPort<int>("self_blood"),
                BT::OutputPort<geometry_msgs::msg::Point>("heal_navigation_point")
            };
        }
    };
}

#endif