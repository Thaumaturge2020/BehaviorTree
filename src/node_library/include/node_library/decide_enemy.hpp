#ifndef RM_SENTRY_2024_DECIDE_ENEMY_
#define RM_SENTRY_2024_DECIDE_ENEMY_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "robot_msgs/msg/autoaim_info.hpp"
#include "robot_msgs/msg/robot_blood_info.hpp"
#include "robot_msgs/msg/walk_cmd.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace BehaviorTree{
     class DecideEnemy:public BT::SyncActionNode{
        private:
        public:
           DecideEnemy(const std::string&name, const BT::NodeConfig& config);
           rclcpp::Subscription<robot_msgs::msg::AutoaimInfo>::SharedPtr subscription_enemy_pos;
           rclcpp::Subscription<robot_msgs::msg::RobotBloodInfo>::SharedPtr subscription_enemy_blood;
           rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr my_pos;
            rclcpp::Node::SharedPtr node1;
            rclcpp::Node::SharedPtr node2;
            rclcpp::Node::SharedPtr node3;
            geometry_msgs::msg::Point My_pos;
            std::vector<robot_msgs::msg::RobotInfo> robot_pos_array;
            std:: vector<robot_msgs::msg::RobotBattleState> robot_blood_array;
            void message_callback_enemy_pos(const robot_msgs::msg::AutoaimInfo &msg);
            void message_callback_enemy_blood(const robot_msgs::msg::RobotBloodInfo &msg);
            void message_callback_my_pos(const nav_msgs::msg::Odometry &msg);
            static BT::PortsList providedPorts(){
                return {
                    BT::InputPort<int>("now_enemy_id"),
                    BT::OutputPort<int>("target_enemy_id")
            };
    }
            BT::NodeStatus tick() override;
    };
}

#endif