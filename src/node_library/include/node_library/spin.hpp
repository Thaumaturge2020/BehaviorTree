#ifndef RM_SENTRY_2024_SPIN_
#define RM_SENTRY_2024_SPIN_


#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include"geometry_msgs/msg/point.h"
#include "nav_msgs/msg/odometry.hpp"
#include "robot_msgs/msg/autoaim_info.hpp"
#include "robot_msgs/msg/walk_cmd.hpp"
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/int32.hpp>
#include"robot_msgs/msg/build_state.hpp"
#include "robot_msgs/msg/robot_blood_info.hpp"


namespace BehaviorTree{
    class Spin:public BT::SyncActionNode{
        private:
        public:
            Spin(const std::string&name, const BT::NodeConfig& config);
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr my_pos;
            rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_spin_able;
            rclcpp::Subscription<robot_msgs::msg::BuildState>::SharedPtr sub_build_state;
            rclcpp::Subscription<robot_msgs::msg::RobotBloodInfo>::SharedPtr sub_self_blood;
            rclcpp::Subscription<robot_msgs::msg::AutoaimInfo>::SharedPtr subscription_enemy_pos;
            rclcpp::Node::SharedPtr node1;
            rclcpp::Node::SharedPtr node2;
            rclcpp::Node::SharedPtr node3;
            rclcpp::Node::SharedPtr node4;
            rclcpp::Node::SharedPtr node5;
            std::vector<robot_msgs::msg::RobotInfo> robot_pos_array;
            std_msgs::msg::Int64 spin_able;
            robot_msgs::msg::BuildState build_state;
            geometry_msgs::msg::Point enemy_pos;
            geometry_msgs::msg::Point My_pos;
            robot_msgs::msg::RobotBloodInfo self_blood;
            void message_callback_enemy_pos(const robot_msgs::msg::AutoaimInfo &msg);
            void message_callback_my_pos(const nav_msgs::msg::Odometry &msg);
            void message_callback_build_state(const robot_msgs::msg::BuildState &msg);
             void message_callback_self_blood(const robot_msgs::msg::RobotBloodInfo &msg);
            static BT::PortsList providedPorts(){
                return {};
            }
            BT::NodeStatus tick() override;
            
    };
}

#endif