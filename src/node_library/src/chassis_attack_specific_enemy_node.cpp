#include "node_library/chassis_attack_specific_enemy_node.hpp"

namespace BehaviorTree
{
    ChassisAttackSpecificEnemyNode::ChassisAttackSpecificEnemyNode(const std::string &name, const BT::NodeConfig &config) :
            BT::SyncActionNode(name, config)
    {
        rclcpp::Time ti_now = rclcpp::Clock().now();
        node1 = rclcpp::Node::make_shared("subscriber_enemy_pos");
        node2 = rclcpp::Node::make_shared("subscriber_enemy_blood");
        subscription_enemy_pos = node1->create_subscription<robot_msgs::msg::AutoaimInfo>("autoaim2decision", 10, std::bind(&ChassisAttackSpecificEnemyNode::message_callback_enemy_pos, this, std::placeholders::_1));
        subscription_enemy_blood = node2->create_subscription<robot_msgs::msg::RobotBloodInfo>("sentry/enemy_blood", 10, std::bind(&ChassisAttackSpecificEnemyNode::message_callback_enemy_blood, this, std::placeholders::_1));
    }

    void ChassisAttackSpecificEnemyNode::message_callback_enemy_pos(const robot_msgs::msg::AutoaimInfo &msg)
    {
        robot_pos_array = msg.data;
        return;
    }

    void ChassisAttackSpecificEnemyNode::message_callback_enemy_blood(const robot_msgs::msg::RobotBloodInfo &msg)
    {
        robot_blood_array = msg.data;
        return;
    }

    BT::NodeStatus ChassisAttackSpecificEnemyNode::tick()
    {
        // RCLCPP_INFO(rclcpp::get_logger("base_attack_specific_enemy_node"), "I'm ticked");
        rclcpp::spin_some(node1);
        rclcpp::spin_some(node2);
        size_t array_size = robot_pos_array.size();
        int require_enemy_id;
        if (!getInput<int>("require_enemy_id", require_enemy_id))
        {
            // RCLCPP_INFO(rclcpp::get_logger("base_attack_specific_enemy_node"), "No Response");
            return BT::NodeStatus::FAILURE;
        }

        for (int i = 0; i < array_size; ++i)
        {
            if (robot_pos_array[i].id == require_enemy_id)
            {
                setOutput<int>("target_enemy_id", require_enemy_id);
                RCLCPP_INFO(rclcpp::get_logger("base_attack_specific_enemy_node"), "Enemy Found");
                return BT::NodeStatus::SUCCESS;
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("base_attack_specific_enemy_node"), "Not Found");
        setOutput<int>("target_enemy_id", -1);
        return BT::NodeStatus::FAILURE;
    }
}

// BT_REGISTER_NODES(factory)
// {
//     factory.registerNodeType<BehaviorTree::ChassisAttackSpecificEnemyNode>("ChassisAttackSpecificEnemyNode");
// }
