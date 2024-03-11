#ifndef RM_SENTRY_2024_BASE_ATTACK_STATIC_
#define RM_SENTRY_2024_BASE_ATTACK_STATIC_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "robot_msgs/msg/geometry_msgs/Point.hpp"
#include "robot_msgs/msg/walk_cmd.hpp"

namespace BehaviorTree{
    class nav_u_node:public BT::SyncActionNode{
        public:
        rclcpp::Publisher<robot_msgs::msg::geometry_msgs::Point>::SharedPtr publisher_decision_pos;
        rclcpp::Node::SharedPtr node1;

        nav_u_node(const std::string&name, const BT::NodeConfig& config);


        static BT::PortsList providedPorts(){
                return {
                    BT::InputPort<robot_msgs::msg::geometry_msgs::Point> ("navigation_point"),
                    
                };
            }
        BT::NodeStatus tick() override;
    }


}

#endif