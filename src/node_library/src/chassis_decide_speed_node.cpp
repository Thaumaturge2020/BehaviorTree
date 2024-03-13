#include "node_library/chassis_decide_speed_node.hpp"

namespace BehaviorTree{

    ChassisDecideSpeedNode::ChassisDecideSpeedNode(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                    node1 = rclcpp::Node::make_shared("subscriber_enemy_pos");
                    subscription_enemy_pos = node1->create_subscription<robot_msgs::msg::AutoaimInfoSpeed>("autoaim2decision",10,std::bind(&ChassisDecideSpeedNode::message_callback_enemy_pos,this,std::placeholders::_1));
                    const auto map_file = toml::parse(ROOT "config/battle_information.toml");
                    highspeed_distance = toml::find<double>(map_file,"highspeed_distance");
                    highspeed_velocity = toml::find<double>(map_file,"highspeed_velocity");
                    hightwist_velocity = toml::find<double>(map_file,"hightwist_velocity");
                }

    void ChassisDecideSpeedNode::message_callback_enemy_pos(const robot_msgs::msg::AutoaimInfoSpeed &msg){
        robot_pos_array = msg.data;
        return;
    }

    BT::NodeStatus ChassisDecideSpeedNode::tick(){
        int target_id;
        if(!getInput<int>("require_enemy_id",target_id)) return BT::NodeStatus::FAILURE;

        for(int i=0,lim = robot_pos_array.size();i<lim;++i){
            if(robot_pos_array[i].id == target_id){
                if(Eigen::Vector3d(robot_pos_array[i].pos.x,robot_pos_array[i].pos.y,robot_pos_array[i].pos.z).norm() < highspeed_distance
                    && Eigen::Vector2d(robot_pos_array[i].vel.x,robot_pos_array[i].vel.y).norm() < highspeed_velocity
                    && robot_pos_array[i].vel.z < hightwist_velocity)
                setOutput<int>("shooting_speed",3);

                
                else if(Eigen::Vector3d(robot_pos_array[i].pos.x,robot_pos_array[i].pos.y,robot_pos_array[i].pos.z).norm() < highspeed_distance
                    && Eigen::Vector2d(robot_pos_array[i].vel.x,robot_pos_array[i].vel.y).norm() < highspeed_velocity)
                setOutput<int>("shooting_speed",2);
                
                else if(Eigen::Vector3d(robot_pos_array[i].pos.x,robot_pos_array[i].pos.y,robot_pos_array[i].pos.z).norm() < highspeed_distance)
                setOutput<int>("shooting_speed",1);
                
                else setOutput<int>("shooting_speed",0);
                
                return BT::NodeStatus::SUCCESS;
            }
        }
        return BT::NodeStatus::FAILURE;
    }
}

// BT_REGISTER_NODES(factory)
// {
//     factory.registerNodeType<BehaviorTree::ChassisDecideSpeedNode>("ChassisDecideSpeedNode");
// }
