#ifndef RM_SENTRY_2024_OPERATOR_GIMBAL_NODE_
#define RM_SENTRY_2024_OPERATOR_GIMBAL_NODE_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "robot_msgs/msg/walk_cmd.hpp"
#include "robot_msgs/msg/autoaim_info.hpp"
#include "robot_msgs/msg/op_cmd.hpp"

namespace BehaviorTree{
    class OperatorGimbalNode:public BT::SyncActionNode{
        private:
        public:
            rclcpp::Subscription<robot_msgs::msg::OpCmd>::SharedPtr subscription_operator_cmd;
            rclcpp::Node::SharedPtr node1;
            int robot_id,flag;
            OperatorGimbalNode(const std::string&name, const BT::NodeConfig& config);
            void message_callback_operator_cmd(const robot_msgs::msg::OpCmd &msg);
            static BT::PortsList providedPorts(){
                return {
                    BT::OutputPort<int>("target_id")
                };
            }
            BT::NodeStatus tick() override;
            
    };
}

#endif