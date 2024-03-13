#ifndef RM_SENTRY_2024_CHASSIS_STATIC_ATTACK_
#define RM_SENTRY_2024_CHASSIS_STATIC_ATTACK_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "robot_msgs/msg/walk_cmd.hpp"
#include "robot_msgs/msg/autoaim_info.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace BehaviorTree{
    class ChassisStaticAttackNode:public BT::SyncActionNode{
        private:
        public:
            rclcpp::Subscription<robot_msgs::msg::AutoaimInfo>::SharedPtr subscription_enemy_pos;
            rclcpp::Node::SharedPtr node1;
            ChassisStaticAttackNode(const std::string&name, const BT::NodeConfig& config);
            std::vector<robot_msgs::msg::RobotInfo> robot_pos_array;
            void message_callback_enemy_pos(const robot_msgs::msg::AutoaimInfo &msg);
            static BT::PortsList providedPorts(){
                return {
                    BT::InputPort<int>("given_id"),
                    BT::OutputPort<int>("target_id")
                };
            }
            BT::NodeStatus tick() override;
    };
}

#endif