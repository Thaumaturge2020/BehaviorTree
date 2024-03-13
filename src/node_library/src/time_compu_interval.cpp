#include "node_library/time_compu_interval.hpp"

namespace BehaviorTree{

    TimeCompuInterval::TimeCompuInterval(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    time = 0;
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                    node = rclcpp::Node::make_shared("time_compu_node");
                    sub_game_time = node->create_subscription<std_msgs::msg::Int32>("/game_time",10,std::bind(&TimeCompuInterval::message_callback_game_time,this,std::placeholders::_1));
                }

    void TimeCompuInterval::message_callback_game_time(const std_msgs::msg::Int32 &msg){
      if(!msg.data){
        return;
      }
       if_start_count = true;
       time_begin = rclcpp::Clock().now();
       return;
    }

    BT::NodeStatus TimeCompuInterval::tick(){
        if(!if_start_count) return BT::NodeStatus::FAILURE;
        RCLCPP_INFO(rclcpp::get_logger("time_compu_node"),"I'm ticked");
        rclcpp::spin_some(node);
        double time_limit_min,time_limit_max = 0;
        if(!getInput("time_limit",time_limit_max)) return BT::NodeStatus::FAILURE;
        if(!getInput("time_limit",time_limit_min)) return BT::NodeStatus::FAILURE;
        double now_ti = (rclcpp::Clock().now() - time_begin).seconds();
        if(now_ti < time_limit_min || now_ti > time_limit_max) return BT::NodeStatus::FAILURE;
        time_begin = rclcpp::Clock().now();
        return BT::NodeStatus::SUCCESS;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<Time_compu::time_compu>("time_compu_node");
// }