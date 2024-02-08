#ifndef RM_SENTRY_2024_NAVIGATION_BUILDING_
#define RM_SENTRY_2024_NAVIGATION_BUILDING_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "robot_msgs/msg/build_state_array.hpp"
#include "robot_msgs/msg/build_state.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace BehaviorTree{
    class NavigationBuildingNode:public BT::SyncActionNode{
        private:
        public:
            rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_pos;
            rclcpp::Node::SharedPtr node1;
            NavigationBuildingNode(const std::string&name, const BT::NodeConfig& config);
            geometry_msgs::msg::Point pos;
            std::vector<std::vector<double>> building_pos;
            static BT::PortsList providedPorts(){
                return {
                    BT::InputPort<int>("now_build_id")
                };
            }
            BT::NodeStatus tick() override;
    };
}

#endif