#ifndef RM_SENTRY_2024_RADAR_DECISION_
#define RM_SENTRY_2024_RADAR_DECISION_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "robot_msgs/msg/autoaim_info.hpp"
#include "geometry_msgs/msg/point.h"
#include "robot_msgs/msg/robot_info.hpp"

namespace BehaviorTree{
    class RadarDecision:public BT::SyncActionNode{
        private:
        public:
            rclcpp::Subscription<robot_msgs::msg::AutoaimInfo>::SharedPtr radar_info;
            rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr robot_position;
            rclcpp::Node::SharedPtr node_radar_independent;
            RadarDecision(const std::string&name, const BT::NodeConfig& config);
            std::vector<robot_msgs::msg::RobotInfo> robot_pos_array;
            geometry_msgs::msg::Point position;
            double distance_limit_radar,distance_limit_enemy,theta_limit;
            double weight_id_enemy[9];
           // double enemy_cover_judge[9][2];
            double weight_distance_radar,weight_distance_enemy,weight_last_choice;
            void message_callback_radar_independent(const robot_msgs::msg::AutoaimInfo &msg);
            void message_callback_robot_position(const geometry_msgs::msg::Point &msg);
            static BT::PortsList providedPorts(){
                return {
                    BT::InputPort<int>("now_enemy_id"),
                    BT::InputPort<double>("time_begin"),
                    BT::OutputPort<int>("target_enemy_id")
                };
            }
            BT::NodeStatus tick() override;
           
    };
}

#endif