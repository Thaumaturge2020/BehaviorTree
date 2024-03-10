#ifndef RM_SENTRY_2024_GIMBAL_CHOOSE_ENEMY_
#define RM_SENTRY_2024_GIMBAL_CHOOSE_ENEMY_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "robot_msgs/msg/autoaim_info.hpp"
#include <vector>

namespace BehaviorTree{
    class GimbalChooseEnemyNode:public BT::SyncActionNode{
        private:
        public:
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_base_angle;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_cam_angle;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_base2cam_angle;
            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_autoaim_able;
            rclcpp::Subscription<robot_msgs::msg::AutoaimInfo>::SharedPtr subscription_enemy_pos;

            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_enemy_info;

            rclcpp::Node::SharedPtr gimbal_choose_enemy_node;
            GimbalChooseEnemyNode(const std::string&name, const BT::NodeConfig& config);
            double base_angle=0,cam_angle=0,base2cam_angle=0;
            std::vector<robot_msgs::msg::RobotInfo> enemy_pos;
            int autoaim_able;
            void message_callback_base_angle(const std_msgs::msg::Float64 &msg);
            void message_callback_cam_angle(const std_msgs::msg::Float64 &msg);
            void message_callback_base2cam_angle(const std_msgs::msg::Float64 &msg);
            void message_callback_autoaim_able(const std_msgs::msg::Int32 &msg);
            void message_callback_enemy_pos(const robot_msgs::msg::AutoaimInfo &msg);
            static BT::PortsList providedPorts(){
                return {
                    BT::InputPort<int>("now_enemy_id"),
                    BT::InputPort<int>("shooting_speed")
                };
            }
            BT::NodeStatus tick() override;
    };
}

#endif