#include "node_library/nav_to_specific_place.hpp"

namespace BehaviorTree{

    NavToSpecificPlace::NavToSpecificPlace(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                    node1 = rclcpp::Node::make_shared("subscriber_enemy_pos");                    
                    publisher_velocity_cmd = node1->create_publisher<geometry_msgs::msg::Point>("decision2ECbasespin",10);
                    flag = 0;
                }

    BT::NodeStatus NavToSpecificPlace::tick(){
        // RCLCPP_INFO(rclcpp::get_logger("NavToSpecificPlace"),"I'm ticked");
        rclcpp::spin_some(node1);
        geometry_msgs::msg::Point target_place;
        if(getInput<geometry_msgs::msg::Point>("target_place",target_place))
        publisher_velocity_cmd->publish(target_place);
        flag = 0;
        return BT::NodeStatus::FAILURE;
    }
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<BehaviorTree::NavToSpecificPlace>("NavToSpecificPlace");
}