#include "node_library/nav_heal_node.hpp"

namespace BehaviorTree{

    nav_heal_node::nav_heal_node(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                    node1 = rclcpp::Node::make_shared("810");    
                    publisher_heal_point = node1->create_publisher<robot_msgs::msg::geometry_msgs::Point>("heal_point",10);
                }

    bool nav_heal_node::cheak_blood()
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
        robot_msgs::msg::geometry_msgs::Point heal_position;//读设置。
        if(cheak_blood())
        {
            publisher_heal_point->publish(heal_position);
            setOutput<robot_msgs::msg::geometry_msgs::Point>("heal_navigation_point",heal_position);
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<BehaviorTree::nav_heal_node>("NavHealNode");
}