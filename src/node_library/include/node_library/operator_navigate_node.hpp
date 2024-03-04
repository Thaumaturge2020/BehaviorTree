#ifndef RM_SENTRY_2024_BASE_ATTACK_STATIC_
#define RM_SENTRY_2024_BASE_ATTACK_STATIC_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "robot_msgs/msg/walk_cmd.hpp"
#include "robot_msgs/msg/autoaim_info.hpp"
#include "robot_msgs/msg/op_cmd.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace BehaviorTree{
    class OperatorNavigateNode:public BT::SyncActionNode{
        private:
        public:
            rclcpp::Subscription<robot_msgs::msg::OpCmd>::SharedPtr subscription_operator_cmd;
            rclcpp::Node::SharedPtr node1;
            geometry_msgs::msg::Point navigate_point;
            bool flag;
            OperatorNavigateNode(const std::string&name, const BT::NodeConfig& config);
            void message_callback_operator_cmd(const robot_msgs::msg::OpCmd &msg);
            static BT::PortsList providedPorts(){
                return {
                    BT::OutputPort<geometry_msgs::msg::Point>("navigation_point")
                };
            }
            BT::NodeStatus tick() override;
            
    };
}

#endif