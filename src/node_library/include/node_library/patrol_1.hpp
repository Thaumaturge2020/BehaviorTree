#ifndef RM_SENTRY_2024_PATROL_1_
#define RM_SENTRY_2024_PATROL_1_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/point.h"
#include "robot_msgs/msg/autoaim_info.hpp"
#include "robot_msgs/msg/robot_info.hpp"

namespace BehaviorTree{
    class PATROL1Node:public BT::SyncActionNode{
        private:
        public:
            rclcpp::Subscription<robot_msgs::msg::AutoaimInfo>::SharedPtr patrol_radar_info;
            rclcpp::Node::SharedPtr node_patrol1;
            PATROL1Node(const std::string&name, const BT::NodeConfig& config);
            int num,clock,situation,new_situation;
            int num_limit[5],clock_limit[5];
            double patrol_1_position[10][3][5],distance;
            double weight_enemy[9];  //0.8 0.8 0.8 0.8 0.8 0.8 0.8 0.8 0.8
            std::vector<robot_msgs::msg::RobotInfo> robot_pos_array;
            void message_callback_patrol_radar_info(const robot_msgs::msg::AutoaimInfo &msg);
            static BT::PortsList providedPorts(){
                return {
                    BT::InputPort<int>("area_choose"),
                    BT::OutputPort<int>("area_choose"),
                    BT::InputPort<double>("time_begin"),
                    BT::InputPort<int>("situation"),
                    BT::OutputPort<geometry_msgs::msg::Point>("navigation_point ")  ////目标点的坐标
                };
            }
            BT::NodeStatus tick() override;
            
    };
}

#endif