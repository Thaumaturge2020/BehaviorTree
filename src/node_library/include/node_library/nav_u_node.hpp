#ifndef RM_SENTRY_2024_NAV_U_NODE_
#define RM_SENTRY_2024_NAV_U_NODE_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/point.hpp"
#include "robot_msgs/msg/walk_cmd.hpp"

namespace BehaviorTree{
    class nav_u_node:public BT::SyncActionNode{
        public:
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_decision_pos;
        rclcpp::Node::SharedPtr node1;

        nav_u_node(const std::string&name, const BT::NodeConfig& config);


        static BT::PortsList providedPorts(){
                return {
                    BT::InputPort<geometry_msgs::msg::Point> ("navigation_point"),
                    
                };
            }
        BT::NodeStatus tick() override;
    };
}

#endif