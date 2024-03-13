#include "node_library/lidar_attack_node.hpp"

namespace BehaviorTree{

    lidar_attack_node::lidar_attack_node(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                    node1 = rclcpp::Node::make_shared("114");                    
                    subscription_enemy_sequnece = node1->create_subscription<robot_msgs::msg::AutoaimInfo>("enemy_lidar_pos",10,std::bind(&lidar_attack_node::enemy_info_receive_callback,this,std::placeholders::_1));
                }

    void lidar_attack_node::enemy_info_receive_callback(const robot_msgs::msg::AutoaimInfo &msg){
        enemy_sequnece = msg.data;
        return;
    }

    bool lidar_attack_node::get_target(int target)
    {
        for(auto DZ:enemy_sequnece)
            {
                if(DZ.id==target)
                {
                    return true;
                }
            }
        return false;
    }

    BT::NodeStatus lidar_attack_node::tick(){
        RCLCPP_INFO(rclcpp::get_logger("lidar_attack_node"),"I'm ticked");
        rclcpp::spin_some(node1);
        int given_id;
        if(!getInput<int>("requires_enemy_id",given_id)){
            RCLCPP_INFO(rclcpp::get_logger("lidar_attack_node"),"No Response");
            return BT::NodeStatus::FAILURE;
        }

        if(get_target(given_id))
        {   
            setOutput<int>("target_enemy_id",given_id);
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::lidar_attack_node>("LidarAttackNode");
// }