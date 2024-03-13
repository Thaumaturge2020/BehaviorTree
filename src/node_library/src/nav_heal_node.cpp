#include "node_library/nav_heal_node.hpp"

namespace BehaviorTree{

    nav_heal_node::nav_heal_node(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                    node1 = rclcpp::Node::make_shared("810");    
                    publisher_heal_point = node1->create_publisher<geometry_msgs::msg::Point>("heal_point",10);
                    const auto toml_file = toml::parse(ROOT "config/battle_information.toml");
                    auto pos = toml::find<std::pair<double,double> >(toml_file,"heal_position");
                    heal_position.x = pos.first;
                    heal_position.y = pos.second;
                    heal_position.z = 0;
                }

    bool nav_heal_node::check_blood()
    {
        int self_blood;
        if(!getInput<int>("self_blood",self_blood))
        {
            return false;
        }
        if(self_blood>300) return false;
        return true;
    }
    

    BT::NodeStatus nav_heal_node::tick()
    {
        if(check_blood())
        {
            publisher_heal_point->publish(heal_position);
            setOutput<geometry_msgs::msg::Point>("heal_navigation_point",heal_position);
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::nav_heal_node>("NavHealNode");
// }