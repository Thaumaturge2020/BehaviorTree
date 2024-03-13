#include "node_library/build_attack_node.hpp"

namespace BehaviorTree{

    build_attack_node::build_attack_node(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                    node1 = rclcpp::Node::make_shared("514");
                    subscriber_build_blood = node1->create_subscription<robot_msgs::msg::BuildState>("build_blood_info",10,std::bind(&build_attack_node::build_blood_recevice_callback,this,std::placeholders::_1));
                }
    
    void build_attack_node::build_blood_recevice_callback(const robot_msgs::msg::BuildState &msg)
    {
        enemy_build_blood = msg.blood;
    }

    bool build_attack_node::target_build_dz()
    {
        if(enemy_build_blood > 0) return true;
        return false;
    }

    BT::NodeStatus build_attack_node::tick()
    {
        RCLCPP_INFO(rclcpp::get_logger("build_attack_node"),"I'm ticked");
        rclcpp::spin_some(node1);
        int given_id;
        getInput<int>("now_build_id",given_id);
        if(target_build_dz()){
            setOutput<int>("target_build_id",given_id);
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }


}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::build_attack_node>("BuildAttackNode");
// }