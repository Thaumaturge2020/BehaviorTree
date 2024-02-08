#include "node_library/time_ctrl.hpp"

namespace BehaviorTree{

    TimeCtrl::TimeCtrl(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    rclcpp::Time ti_now = rclcpp::Clock().now();              
                }

    BT::NodeStatus TimeCtrl::tick(){
        RCLCPP_INFO(rclcpp::get_logger("TimeCtrl_node"),"I'm ticked");
        int now_game_time;
        if(!getInput<int>("now_game_time",now_game_time)){
            RCLCPP_INFO(rclcpp::get_logger("TimeCtrl_node"),"No Response");
            return BT::NodeStatus::FAILURE;
        }
        else if(now_game_time >= 120&&now_game_time <= 180){
            setOutput<int>("game_states",1);
        }
        else if(now_game_time >= 180&&now_game_time <= 300){
            setOutput<int>("game_states",2);
        }
        else if(now_game_time >= 300){
            setOutput<int>("game_states",3);
        }
        else{
            setOutput<int>("game_states",0);
        }
        return BT::NodeStatus::SUCCESS;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<TimeCtrl::TimeCtrl>("TimeCtrl");
// }