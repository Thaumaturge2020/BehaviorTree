#include "node_library/patrol_2.hpp"

namespace BehaviorTree{

    PATROL2Node::PATROL2Node(const std::string& name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    //limit_distance[9]={}
                }

    BT::NodeStatus PATROL2Node::tick(){
        if(getInput<double>("time_during").value()<0.00001)return BT::NodeStatus::FAILURE;
        now_enemy_id=getInput<int>("now_enemy_id").value();
        now_navigation_point=getInput<geometry_msgs::msg::Point>("now_navigation_point").value();
        navigation_point=getInput<geometry_msgs::msg::Point>("navigation_point").value();
        distance=(now_navigation_point.x-navigation_point.x)*(now_navigation_point.x-navigation_point.x)
        +(now_navigation_point.y-navigation_point.y)*(now_navigation_point.y-navigation_point.y)
        +(now_navigation_point.z-navigation_point.z)*(now_navigation_point.z-navigation_point.z);
        if(distance>=limit_distance[now_enemy_id%100])  
        {
            self_point=getInput<geometry_msgs::msg::Point>("self_point").value();
            distance_self_now_navigation_point=(self_point.x-now_navigation_point.x)*(self_point.x-now_navigation_point.x)
            +(self_point.y-now_navigation_point.y)*(self_point.y-now_navigation_point.y)
            +(self_point.z-now_navigation_point.z)*(self_point.z-now_navigation_point.z);
            int enemy_blood=getInput<int>("enemy_blood").value();
            if(100.0/(distance_self_now_navigation_point*(double)enemy_blood)*weight_id_enemy[now_enemy_id%100]>limit)
                return BT::NodeStatus::FAILURE;
            setOutput<geometry_msgs::msg::Point>("now_navigation_point",navigation_point);
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;

}
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::PATROL2Node>("PATROL2Node");
// }