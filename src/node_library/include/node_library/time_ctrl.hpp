#ifndef RM_SENTRY_2024_TIME_CTRL_
#define RM_SENTRY_2024_TIME_CTRL_


#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "std_msgs/msg/int32.hpp"
#include <time.h>


namespace BehaviorTree{
    class TimeCtrl:public BT::SyncActionNode{
        private:
            bool if_start_count = false;
        public:
            TimeCtrl(const std::string&name, const BT::NodeConfig& config);
            static BT::PortsList providedPorts(){
                return {
                    BT::InputPort<int>("now_game_time"),
                    BT::OutputPort<int>("game_states"),
                };
            }
            BT::NodeStatus tick() override;
            
    };
}

#endif