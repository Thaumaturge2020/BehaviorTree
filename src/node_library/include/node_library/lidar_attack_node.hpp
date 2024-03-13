#ifndef RM_SENTRY_2024_LIDAR_ATTACK_NODE_
#define RM_SENTRY_2024_LIDAR_ATTACK_NODE_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "robot_msgs/msg/walk_cmd.hpp"
#include "robot_msgs/msg/autoaim_info.hpp"

namespace BehaviorTree{
    class lidar_attack_node:public BT::SyncActionNode{
        private:
        public:
            rclcpp::Subscription<robot_msgs::msg::AutoaimInfo>::SharedPtr subscription_enemy_sequnece;
            rclcpp::Node::SharedPtr node1;
            std::vector<robot_msgs::msg::RobotInfo> enemy_sequnece;
            lidar_attack_node(const std::string&name, const BT::NodeConfig& config);
            
            void enemy_info_receive_callback(const robot_msgs::msg::AutoaimInfo &msg);
            bool get_target(int target);
            static BT::PortsList providedPorts(){
                return {
                    BT::InputPort<int> ("requires_enemy_id"),
                    BT::OutputPort<int> ("target_enemy_id"),
                };
            }
            BT::NodeStatus tick() override;
            
    };
}

#endif