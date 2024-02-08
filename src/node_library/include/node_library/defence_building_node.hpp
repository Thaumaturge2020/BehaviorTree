#ifndef RM_SENTRY_2024_DEFENCE_BUILDING_
#define RM_SENTRY_2024_DEFENCE_BUILDING_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "robot_msgs/msg/build_state_array.hpp"
#include "robot_msgs/msg/build_state.hpp"

namespace BehaviorTree{
    class DefenceBuildingNode:public BT::SyncActionNode{
        private:
        public:
            rclcpp::Subscription<robot_msgs::msg::BuildStateArray>::SharedPtr subscription_build_state;
            rclcpp::Node::SharedPtr node1;
            DefenceBuildingNode(const std::string&name, const BT::NodeConfig& config);
            std::vector<robot_msgs::msg::BuildState> build_state_array;
            void message_callback_build_state(const robot_msgs::msg::BuildStateArray &msg);
            static BT::PortsList providedPorts(){
                return {
                    BT::InputPort<int>("building_id")
                };
            }
            BT::NodeStatus tick() override;
    };
}

#endif