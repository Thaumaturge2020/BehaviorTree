#ifndef RM_SENTRY_2024_BASE_ATTACK_STATIC_
#define RM_SENTRY_2024_BASE_ATTACK_STATIC_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "robot_msgs/msg/walk_cmd.hpp"
#include "robot_msgs/msg/build_state.hpp"

namespace BehaviorTree{
    class build_attack_node:public BT::SyncActionNode{
        private:
        public:
            rclcpp:Subscription<robot_msgs::msg::buildState>::SharedPtr subscriber_build_blood;
            int enemy_build_blood;
            rclcpp:Node::SharedPtr node1;

            build_attack_node(const std::string&name, const BT::NodeConfig& config);

            void build_blood_recevice_callback(robot_msgs::msg::Build_State &msg);

            static BT::PortsList providedPorts()
            {
                return  
                {
                    BT::InputPort<int>("now_build_id"),
                    BT::OutputPort<int>("target_build_id"),
                };
            }

            bool target_build_dz();

            BT::NodeStatus tick() override;
    }
}











#endif