#ifndef RM_SENTRY_2024_TIME_COMPU_INTERVAL_
#define RM_SENTRY_2024_TIME_COMPU_INTERVAL_


#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "std_msgs/msg/int32.hpp"
#include <time.h>


namespace BehaviorTree{
    class TimeCompuInterval:public BT::SyncActionNode{
        private:
            bool if_start_count = false;
            int time;
        public:
            TimeCompuInterval(const std::string&name, const BT::NodeConfig& config);
            rclcpp::TimerBase::SharedPtr timer1;
            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_game_time;
            rclcpp::Node::SharedPtr node;
            rclcpp::Time time_begin;
            void message_callback_game_time(const std_msgs::msg::Int32 &msg);
            void timer1_callback();
            static BT::PortsList providedPorts(){
                return {
                    BT::InputPort<double>("time_limit_min"),
                    BT::InputPort<double>("time_limit_max"),
                    BT::OutputPort<double>("now_game_time"),
                };
            }
            BT::NodeStatus tick() override;
            
    };
}

#endif