#include "node_library/operator_navigate_node.hpp"

namespace BehaviorTree{

    OperatorNavigateNode::OperatorNavigateNode(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                    node1 = rclcpp::Node::make_shared("subscriber_enemy_pos");                    
                    subscription_operator_cmd = node1->create_subscription<robot_msgs::msg::OpCmd>("operator_cmd",10,std::bind(&OperatorNavigateNode::message_callback_operator_cmd,this,std::placeholders::_1));
                    flag = 0;
                }

    void OperatorNavigateNode::message_callback_operator_cmd(const robot_msgs::msg::OpCmd &msg){
        navigate_point = msg.pos;
        flag = 1;
        return;
    }

    BT::NodeStatus OperatorNavigateNode::tick(){
        RCLCPP_INFO(rclcpp::get_logger("base_attack_node"),"I'm ticked");
        rclcpp::spin_some(node1);

        if(flag)
        setOutput<geometry_msgs::msg::Point>("navigation_point",navigate_point);
        flag = 0;

        return BT::NodeStatus::FAILURE;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::OperatorNavigateNode>("OperatorNavigateNode");
// }