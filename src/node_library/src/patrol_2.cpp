#include "node_library/patrol_2.hpp"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "nav_msgs/msg/odometry.hpp"

namespace BehaviorTree{

    Patrol2Node::Patrol2Node(const std::string& name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    //limit_distance[9]={}
                    RCLCPP_INFO(rclcpp::get_logger("patrol_2_node"),"Patro2Node initialized");
                    state = 0;
                }

    BT::NodeStatus Patrol2Node::tick(){
        double distance_limit_max,distance_limit_min;
        if(!getInput<double>("distance_limit_max",distance_limit_max)
        || !getInput<double>("distance_limit_min",distance_limit_min))
        return BT::NodeStatus::SUCCESS;
        getInput<geometry_msgs::msg::Point>("self_point",self_point);
        getInput<geometry_msgs::msg::Point>("navigation_point",navigation_point);
        Eigen::Vector3d self_eigen,self_navigation;
        self_eigen = Eigen::Vector3d(self_point.x,self_point.y,self_point.z);
        self_navigation = Eigen::Vector3d(navigation_point.x,navigation_point.y,navigation_point.z);
        distance = (self_eigen - self_navigation).norm();

        nav_msgs::msg::Odometry A_Point;

        if(state == 0){
            if(distance>=distance_limit_max)
            state = 1;
        }
        if(state == 1){
            if(distance>=distance_limit_min){
                setOutput<geometry_msgs::msg::Point>("now_navigation_point",navigation_point);
            }
            else state = 0;
        }
        return BT::NodeStatus::SUCCESS;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::Patrol2Node>("Patrol2Node");
// }