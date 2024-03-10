#include "node_library/time_compu.hpp"

namespace BehaviorTree{

    TimeCompuLimit::TimeCompu(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    time = 0;
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                    node = rclcpp::Node::make_shared("time_compu_node");
                    sub_game_time = node->create_subscription<std_msgs::msg::Int32>("/game_time",10,std::bind(&TimeCompu::message_callback_game_time,this,std::placeholders::_1));
                }

    void TimeCompuLimit::message_callback_game_time(const std_msgs::msg::Int32 &msg){
      if(!msg.data){
        return;
      }
       if_start_count = true;
       time_begin = rclcpp::Clock().now();
       return;
    }

    BT::NodeStatus TimeCompuLimit::tick(){
        if(!if_start_count) return BT::NodeStatus::FAILURE;
        RCLCPP_INFO(rclcpp::get_logger("time_compu_node"),"I'm ticked");
        rclcpp::spin_some(node);
        double ti_limit = 0;
        if(!getInput("time_limit",ti_limit)) return BT::NodeStatus::FAILURE;
        double now_ti = (rclcpp::Clock().now() - time_begin).seconds();
        if(now_ti < ti_limit) return BT::NodeStatus::FAILURE;
        time_begin = rclcpp::Clock().now();
        return BT::NodeStatus::SUCCESS;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<Time_compu::time_compu>("time_compu_node");
// }