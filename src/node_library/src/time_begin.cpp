#include "node_library/time_begin.hpp"

namespace BehaviorTree{
    TimeBegin::TimeBegin(const std::string& name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    start_time = rclcpp::Clock().now();
                    setOutput<double>("time_during",0.0);
                    node_time_begin = rclcpp::Node::make_shared("node_time_begin");
                    time_begin_subscription = node_time_begin->create_subscription<std_msgs::msg::Int32>("game_begin",10,std::bind(&TimeBegin::message_callback_time_begin,this,std::placeholders::_1));
                }

void TimeBegin::message_callback_time_begin(const std_msgs::msg::Int32 &msg){

    now_time = rclcpp::Clock().now();
    during = now_time.seconds()-start_time.seconds();
    setOutput<double>("time_begin",now_time.seconds());
    setOutput<double>("time_during",during);
    return;
}

BT::NodeStatus BehaviorTree::TimeBegin:: tick(){
    rclcpp::spin_some(node_time_begin);
    if(getInput<double>("time_during").value()-0<0.00001) return BT::NodeStatus::FAILURE;

    return BT::NodeStatus::SUCCESS;

}


}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::TimeBegin>("TimeBegin");
// }