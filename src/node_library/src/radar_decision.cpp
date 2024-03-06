#include "node_library/radar_decision.hpp"
#include "math.h"

namespace BehaviorTree{


    RadarDecision::RadarDecision(const std::string& name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    //初始化 distance_limit_radar,distance_limit_enemy,weight_distance_radar,weight_distance_enemy,weight_last_choice
                    //
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                    node_radar_independent = rclcpp::Node::make_shared("chassis_independent");                    
                    radar_info = node_radar_independent->create_subscription<robot_msgs::msg::AutoaimInfo>("radar_info",10,std::bind(&RadarDecision::message_callback_radar_independent,this,std::placeholders::_1));
                    robot_position= node_radar_independent->create_subscription<geometry_msgs::msg::Point>("robot_position",10,std::bind(&RadarDecision::message_callback_robot_position,this,std::placeholders::_1));
                }

    void RadarDecision::message_callback_radar_independent(const robot_msgs::msg::AutoaimInfo &msg){
        robot_pos_array = msg.data;
        return;
    }

    void RadarDecision::message_callback_robot_position(const geometry_msgs::msg::Point &msg){
        position=msg;
        return;
    }

     BT::NodeStatus BehaviorTree::RadarDecision:: tick(){
        rclcpp::spin_some(node_radar_independent);
        if(getInput<double>("time_begin").value()>rclcpp::Clock().now().seconds())return BT::NodeStatus::FAILURE;
        double weight_enemy[9]={0};
        size_t array_size=robot_pos_array.size();
        double distance=(robot_pos_array[0].pos.x-position.x)*(robot_pos_array[0].pos.x-position.x)+(robot_pos_array[0].pos.y-position.y)*(robot_pos_array[0].pos.y-position.y)+(robot_pos_array[0].pos.z-position.z)*(robot_pos_array[0].pos.z-position.z);
        int number=0;
        for(int i=0;i<array_size;i++){
            for(int j=0;j<array_size;j++)
            {
                if(i!=j){
                    distance = (robot_pos_array[i].pos.x-robot_pos_array[j].pos.x) * (robot_pos_array[i].pos.x-robot_pos_array[j].pos.x)
                    + (robot_pos_array[i].pos.y-robot_pos_array[j].pos.y) * (robot_pos_array[i].pos.y-robot_pos_array[j].pos.y)
                    + (robot_pos_array[i].pos.z-robot_pos_array[j].pos.z) * (robot_pos_array[i].pos.z-robot_pos_array[j].pos.z);
                    if(distance<=distance_limit_enemy)
                    weight_enemy[robot_pos_array[i].id%100]+=100.0/distance*weight_distance_enemy*weight_id_enemy[robot_pos_array[j].id%100];
                }
            }
            distance=(robot_pos_array[i].pos.x-position.x)*(robot_pos_array[i].pos.x-position.x)
            +(robot_pos_array[i].pos.y-position.y)*(robot_pos_array[i].pos.y-position.y)
            +(robot_pos_array[i].pos.z-position.z)*(robot_pos_array[i].pos.z-position.z);
            weight_enemy[robot_pos_array[i].id%100]+=100.0/distance*weight_distance_radar*weight_id_enemy[robot_pos_array[i].id%100];
                
           /* if (distance>(robot_pos_array[0].pos.x-position.x)*(robot_pos_array[0].pos.x-position.x)+(robot_pos_array[0].pos.y-position.y)*(robot_pos_array[0].pos.y-position.y)+(robot_pos_array[0].pos.z-position.z)*(robot_pos_array[0].pos.z-position.z)){
                distance=(robot_pos_array[0].pos.x-position.x)*(robot_pos_array[0].pos.x-position.x)+(robot_pos_array[0].pos.y-position.y)*(robot_pos_array[0].pos.y-position.y)+(robot_pos_array[0].pos.z-position.z)*(robot_pos_array[0].pos.z-position.z);
                number=robot_pos_array[i].id;
            }*/
            
            
        }
        /*加一个判断是否在可攻击范围内，下班了*/
        int last_choice_id=getInput<int>("now_enemy_id").value();
        weight_enemy[last_choice_id%100]+=weight_last_choice;
        for(int i=1;i<9;i++)
        weight_enemy[number]>weight_enemy[i]?number:i;
        setOutput<int>("target_enemy_id",number+100);
        return BT::NodeStatus::SUCCESS;
    }


}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::RadarDecision>("RadarDecision");
// }
