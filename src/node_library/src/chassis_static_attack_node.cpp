#include "node_library/chassis_static_attack_node.hpp"

namespace BehaviorTree{

    ChassisStaticAttackNode::ChassisStaticAttackNode(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                    node1 = rclcpp::Node::make_shared("subscriber_enemy_pos");                    
                    subscription_enemy_pos = node1->create_subscription<robot_msgs::msg::AutoaimInfo>("autoaim2decision",10,std::bind(&ChassisStaticAttackNode::message_callback_enemy_pos,this,std::placeholders::_1));
                }

    void ChassisStaticAttackNode::message_callback_enemy_pos(const robot_msgs::msg::AutoaimInfo &msg){
        robot_pos_array = msg.data;
        return;
    }

    BT::NodeStatus ChassisStaticAttackNode::tick(){
        RCLCPP_INFO(rclcpp::get_logger("base_attack_node"),"I'm ticked");
        rclcpp::spin_some(node1);
        size_t array_size = robot_pos_array.size();
        int given_id;
        if(!getInput<int>("given_id",given_id)){
            RCLCPP_INFO(rclcpp::get_logger("base_attack_node"),"No Response");
            return BT::NodeStatus::FAILURE;
        }

        for(int i=0;i<array_size;++i){
            if(robot_pos_array[i].id == given_id){
                setOutput<int>("target_id",given_id);
                return BT::NodeStatus::SUCCESS;
            }
        }
        return BT::NodeStatus::FAILURE;
    }
}

// BT_REGISTER_NODES(factory)
// {
//     factory.registerNodeType<BehaviorTree::ChassisStaticAttackNode>("ChassisStaticAttackNode");
// }

