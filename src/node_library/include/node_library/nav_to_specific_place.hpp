#ifndef RM_SENTRY_2024_BASE_ATTACK_STATIC_
#define RM_SENTRY_2024_BASE_ATTACK_STATIC_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "robot_msgs/msg/walk_cmd.hpp"
#include "robot_msgs/msg/autoaim_info.hpp"
#include "robot_msgs/msg/op_cmd.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/wrench.hpp"

namespace BehaviorTree{
    class NavToSpecificPlace:public BT::SyncActionNode{
        private:
        public:
            rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_velocity_cmd;
            rclcpp::Node::SharedPtr node1;
            int robot_id,flag;
            NavToSpecificPlace(const std::string&name, const BT::NodeConfig& config);
            void message_callback_operator_cmd(const robot_msgs::msg::OpCmd &msg);
            static BT::PortsList providedPorts(){
                return {
                    BT::InputPort<geometry_msgs::msg::Point>("target_place")
                };
            }
            BT::NodeStatus tick() override;
            
    };
}

#endif