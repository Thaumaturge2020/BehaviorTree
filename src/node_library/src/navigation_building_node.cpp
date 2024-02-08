#include "node_library/navigation_building_node.hpp"
#include "toml.hpp"

namespace BehaviorTree
{
    const auto map_data = toml::parse(ROOT "config/battle_information.toml");
    NavigationBuildingNode::NavigationBuildingNode(const std::string &name, const BT::NodeConfig &config):
        BT::SyncActionNode(name, config)
    {
        rclcpp::Time ti_now = rclcpp::Clock().now();
        node1 = rclcpp::Node::make_shared("publisher_pos");
        publisher_pos = node1->create_publisher<geometry_msgs::msg::Point>("decision2nav_pos", 10);
        building_pos = toml::find<std::vector<std::vector<double>>>(map_data,"BUILDING_POS");
    }
    
    BT::NodeStatus NavigationBuildingNode::tick()
    {
        // RCLCPP_INFO(rclcpp::get_logger("navigation_building_node"), "I'm ticked");
        int building_id;
        if (!getInput<int>("now_build_id", building_id))
        {
            // RCLCPP_INFO(rclcpp::get_logger("navigation_building_node"), "No Response");
            return BT::NodeStatus::FAILURE;
        }
        if(building_id!=0 && building_id!=1 && building_id!=101 && building_id!=100) {
            RCLCPP_INFO(rclcpp::get_logger("navigation_building_node"), "Wrong Input");
            return BT::NodeStatus::FAILURE;
        }
        pos.x = building_pos[building_id%100+building_id/100*2][0]; pos.y=building_pos[building_id%100+building_id/100*2][1]; pos.z=building_pos[building_id%100+building_id/100*2][2];
        publisher_pos->publish(pos);
        RCLCPP_INFO(rclcpp::get_logger("navigation_building_node"), "I'm published,the x is %f,the y is %f,the z is %f\n",pos.x,pos.y,pos.z);
        return BT::NodeStatus::SUCCESS;
    }
}

// BT_REGISTER_NODES(factory)
// {
//     factory.registerNodeType<BehaviorTree::NavigationBuildingNode>("NavigationBuildingNode");
// }