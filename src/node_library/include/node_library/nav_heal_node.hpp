#ifndef RM_SENTRY_2024_BASE_ATTACK_STATIC_
#define RM_SENTRY_2024_BASE_ATTACK_STATIC_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "robot_msgs/msg/geometry_msgs/Point.hpp"

namespace BehaviorTree{
    class nav_heal_node:public BT::SyncActionNode{
        public:
        rclcpp::Node::SharedPtr node1;
        rclcpp::Publisher<robot_msgs::msg::geometry_msgs::Point>::SharedPtr publisher_heal_point;

        nav_heal_node(const std::string&name, const BT::NodeConfig& config);

        BT::NodeStatus tick() override;

        
        
        
        
        bool cheak_blood();

        static BT::PortsList providedPorts()
        {
            return{
                InputPort<int>("self_blood");
                OutputPort<robot_msgs::msg::geometry_msgs::Point>("heal_navigation_point");
            }
        }
    }
}

#endif