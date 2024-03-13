#ifndef RM_SENTRY_2024_CHASSIS_DECIDE_SPEED_NODE_
#define RM_SENTRY_2024_CHASSIS_DECIDE_SPEED_NODE_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "Eigen/Dense"
#include "Eigen/Core"
#include "robot_msgs/msg/robot_blood_info.hpp"
#include "robot_msgs/msg/autoaim_info.hpp"
#include "robot_msgs/msg/autoaim_info_speed.hpp"
#include "toml.hpp"

namespace BehaviorTree{
    class ChassisDecideSpeedNode:public BT::SyncActionNode{
        private:
        public:
            rclcpp::Subscription<robot_msgs::msg::AutoaimInfoSpeed>::SharedPtr subscription_enemy_pos;
            rclcpp::Subscription<robot_msgs::msg::RobotBloodInfo>::SharedPtr subscription_enemy_blood;
            rclcpp::Node::SharedPtr node1;
            rclcpp::Node::SharedPtr node2;
            double highspeed_distance,highspeed_velocity,hightwist_velocity;
            ChassisDecideSpeedNode(const std::string&name, const BT::NodeConfig& config);
            std::vector<robot_msgs::msg::RobotInfoSpeed> robot_pos_array;
            std::vector<robot_msgs::msg::RobotBattleState> robot_blood_array;
            void message_callback_enemy_pos(const robot_msgs::msg::AutoaimInfoSpeed &msg);
            void message_callback_enemy_blood(const robot_msgs::msg::RobotBloodInfo &msg);
            static BT::PortsList providedPorts(){
                return {
                    BT::InputPort<int>("require_enemy_id"),
                    BT::OutputPort<int>("shooting_speed")
                };
            }
            BT::NodeStatus tick() override;
    };
}

#endif