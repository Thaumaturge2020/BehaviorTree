#ifndef RM_SENTRY_2024_TIME_COMPU_LIMIT_
#define RM_SENTRY_2024_TIME_COMPU_LIMIT_


#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "std_msgs/msg/int32.hpp"
#include <time.h>
#include "toml.hpp"

namespace BehaviorTree{
    class TimeCompuLimit:public BT::SyncActionNode{
        private:
            bool if_start_count = false;
            int time;
        public:
            TimeCompuLimit(const std::string&name, const BT::NodeConfig& config);
            rclcpp::TimerBase::SharedPtr timer1;
            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_game_time;
            rclcpp::Node::SharedPtr node;
            rclcpp::Time time_begin;
            int time_switch;
            void message_callback_game_time(const std_msgs::msg::Int32 &msg);
            void timer1_callback();
            static BT::PortsList providedPorts(){
                return {
                    BT::InputPort<double>("time_limit1"),
                    BT::InputPort<double>("time_limit2"),
                    BT::InputPort<double>("first_interval"),
                    BT::InputPort<bool>("time_switch"),
                    BT::OutputPort<double>("now_game_time"),
                };
            }
            BT::NodeStatus tick() override;
            
    };
}

#endif