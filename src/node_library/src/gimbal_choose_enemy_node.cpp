#include "node_library/gimbal_choose_enemy_node.hpp"

namespace BehaviorTree
{
    GimbalChooseEnemyNode::GimbalChooseEnemyNode(const std::string &name, const BT::NodeConfig &config) : BT::SyncActionNode(name, config)
    {
        rclcpp::Time ti_now = rclcpp::Clock().now();

        gimbal_choose_enemy_node = rclcpp::Node::make_shared("gimbal_choose_enemy_node");

        subscription_base_angle = gimbal_choose_enemy_node->create_subscription<std_msgs::msg::Float64>("EC2DS/base", 10, std::bind(&GimbalChooseEnemyNode::message_callback_base_angle, this, std::placeholders::_1));
        subscription_cam_angle = gimbal_choose_enemy_node->create_subscription<std_msgs::msg::Float64>("EC2DS/cam", 10, std::bind(&GimbalChooseEnemyNode::message_callback_cam_angle, this, std::placeholders::_1));
        subscription_base2cam_angle = gimbal_choose_enemy_node->create_subscription<std_msgs::msg::Float64>("EC2DS/base2cam", 10, std::bind(&GimbalChooseEnemyNode::message_callback_base2cam_angle, this, std::placeholders::_1));
        subscription_autoaim_able = gimbal_choose_enemy_node->create_subscription<std_msgs::msg::Int32>("/vitual_mode", 10, std::bind(&GimbalChooseEnemyNode::message_callback_autoaim_able, this, std::placeholders::_1));
        subscription_enemy_pos = gimbal_choose_enemy_node->create_subscription<robot_msgs::msg::AutoaimInfo>("/autoaim2decision", 10, std::bind(&GimbalChooseEnemyNode::message_callback_enemy_pos, this, std::placeholders::_1));
        publisher_enemy_info = gimbal_choose_enemy_node->create_publisher<std_msgs::msg::Float64>("decision2autoaim", 10);
    }

    void GimbalChooseEnemyNode::message_callback_base_angle(const std_msgs::msg::Float64 &msg)
    {
        base_angle = msg.data;
        return;
    }

    void GimbalChooseEnemyNode::message_callback_cam_angle(const std_msgs::msg::Float64 &msg)
    {
        cam_angle = msg.data;
        return;
    }

    void GimbalChooseEnemyNode::message_callback_base2cam_angle(const std_msgs::msg::Float64 &msg)
    {
        base2cam_angle = msg.data;
        return;
    }

    void GimbalChooseEnemyNode::message_callback_autoaim_able(const std_msgs::msg::Int32 &msg)
    {
        autoaim_able = msg.data;
        return;
    }

    void GimbalChooseEnemyNode::message_callback_enemy_pos(const robot_msgs::msg::AutoaimInfo &msg){
        enemy_pos = msg.data;
        // RCLCPP_INFO(rclcpp::get_logger("GimbalChooseEnemyNode"), "Successfully callback.");
    }

    BT::NodeStatus GimbalChooseEnemyNode::tick()
    {
        RCLCPP_INFO(rclcpp::get_logger("GimbalChooseEnemyNode"), "I'm ticked");
        rclcpp::spin_some(gimbal_choose_enemy_node);
        //
        publisher_enemy_info->publish(std_msgs::msg::Float64());
        if (!autoaim_able)
        {
            // RCLCPP_INFO(rclcpp::get_logger("GimbalChooseEnemyNode"), "ATTACK OFF");
            return BT::NodeStatus::FAILURE;
        }
        else
        {
            // RCLCPP_INFO(rclcpp::get_logger("GimbalChooseEnemyNode"), "ATTACK ON");
        }
        int now_enemy_id;
        if (!getInput<int>("now_enemy_id", now_enemy_id))
        {
            // RCLCPP_INFO(rclcpp::get_logger("GimbalChooseEnemyNode"), "No Response");
            return BT::NodeStatus::FAILURE;
        }
        // ���˵�λ�� enemy_pos�����̵�λ�� base_pos���Լ���̨������ϵ gimbal_frame    ?????????????

        // // �����������ڵ��̵�λ��
        // auto relative_pos = enemy_pos - base_pos;

        // // �����λ��ת������̨����ϵ
        // auto gimbal_pos = gimbal_frame * relative_pos;

        // // ���㷽λ��
        // double azimuth = std::atan2(gimbal_pos.y, gimbal_pos.x);

        double my_pos_angle = 0;

        RCLCPP_INFO(rclcpp::get_logger("GimbalChooseEnemyNode"), "%d", now_enemy_id);

        for(int i=0,size = enemy_pos.size();i<size;++i){
            if(enemy_pos[i].id == now_enemy_id){
                my_pos_angle = atan2(enemy_pos[i].pos.y,enemy_pos[i].pos.x);
                goto S;
            }
        }

        // RCLCPP_INFO(rclcpp::get_logger("GimbalChooseEnemyNode"), "No Detect Enemy");

        return BT::NodeStatus::FAILURE;

        S:;
        double azimuth = my_pos_angle + cam_angle - base_angle - base2cam_angle;
        // �����λ��
        RCLCPP_INFO(rclcpp::get_logger("GimbalChooseEnemyNode"), "Azimuth: %f", azimuth);
        return BT::NodeStatus::SUCCESS;
    }
}

// BT_REGISTER_NODES(factory)
// {
//     factory.registerNodeType<BehaviorTree::GimbalChooseEnemyNode>("GimbalChooseEnemyNode");
// }
