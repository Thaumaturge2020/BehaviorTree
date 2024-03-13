#include "node_library/defence_building_node.hpp"

namespace BehaviorTree
{
    DefenceBuildingNode::DefenceBuildingNode(const std::string &name, const BT::NodeConfig &config) :
            BT::SyncActionNode(name, config)
    {
        rclcpp::Time ti_now = rclcpp::Clock().now();
        node1 = rclcpp::Node::make_shared("subscriber_build_id");
        subscription_build_state = node1->create_subscription<robot_msgs::msg::BuildStateArray>("now_state_array", 10, std::bind(&DefenceBuildingNode::message_callback_build_state, this, std::placeholders::_1));
    }

    void DefenceBuildingNode::message_callback_build_state(const robot_msgs::msg::BuildStateArray &msg)
    {
        build_state_array = msg.data;
        return;
    }


    BT::NodeStatus DefenceBuildingNode::tick()
    {
        // RCLCPP_INFO(rclcpp::get_logger("defence_building_node"), "I'm ticked");
        rclcpp::spin_some(node1);
        size_t array_size = build_state_array.size();
        int building_id;
        if (!getInput<int>("building_id", building_id))
        {
            // RCLCPP_INFO(rclcpp::get_logger("defence_building_node"), "No Response");
            return BT::NodeStatus::FAILURE;
        }

        for (int i = 0; i < array_size; ++i)
        {
            if (build_state_array[i].id == building_id)
            {
                if(build_state_array[i].blood <= 0){
                    RCLCPP_INFO(rclcpp::get_logger("defence_building_node"), "building is dead");
                    return BT::NodeStatus::FAILURE;
                }
                
                RCLCPP_INFO(rclcpp::get_logger("defence_building_node"), "building Found");
                return BT::NodeStatus::SUCCESS;
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("defence_building_node"), "Not Found");
        return BT::NodeStatus::FAILURE;
    }
}

// BT_REGISTER_NODES(factory)
// {
//     factory.registerNodeType<BehaviorTree::DefenceBuildingNode>("DefenceBuildingNode");
// }