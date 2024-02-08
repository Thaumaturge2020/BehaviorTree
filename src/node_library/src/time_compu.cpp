#include "node_library/time_compu.hpp"

namespace BehaviorTree{

    TimeCompu::TimeCompu(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                    node1 = rclcpp::Node::make_shared("subscriber_game_time"); 
                    node2 = rclcpp::Node::make_shared("time_pub");                   
                    sub_game_time = node1->create_subscription<std_msgs::msg::Int32>("/game_time",10,std::bind(&TimeCompu::message_callback_game_time,this,std::placeholders::_1));
                    timer1 = node2->create_wall_timer(std::chrono::milliseconds(250), std::bind(&TimeCompu::timer1_callback, this));
                }

    void TimeCompu::message_callback_game_time(const std_msgs::msg::Int32 &msg){
      if(!msg.data){
        return;
      }
       if_start_count = true;
       return;
    }

    void TimeCompu::timer1_callback(){
      if(if_start_count){  
       static int i = 0;
       static int time = 0;
	   i++;
	   if(i>=4){
		   i=0;
		   time++;
           setOutput<int>("now_game_time",time);
			}
            return;
		}
        return;
    }

    BT::NodeStatus TimeCompu::tick(){
        RCLCPP_INFO(rclcpp::get_logger("time_compu_node"),"I'm ticked");
        rclcpp::spin_some(node1);
        rclcpp::spin_some(node2);
        return BT::NodeStatus::SUCCESS;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<Time_compu::time_compu>("time_compu_node");
// }