#ifndef RM_SENTRY_2024_PATROL_2_
#define RM_SENTRY_2024_PATROL_2_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/point.h"
#include "robot_msgs/msg/autoaim_info.hpp"
#include "robot_msgs/msg/robot_info.hpp"

namespace BehaviorTree{
    class Patrol2Node:public BT::SyncActionNode{
        private:
        public:
        Patrol2Node(const std::string&name, const BT::NodeConfig& config);
        int now_enemy_id;
        geometry_msgs::msg::Point now_navigation_point, navigation_point, self_point;
        double distance;
        double distance_self_now_navigation_point,limit;
        double weight_id_enemy[9];
        bool state;
        double limit_distance[9];
            static BT::PortsList providedPorts(){
                return {
                    BT::InputPort<double>("distance_limit"),
                    BT::InputPort<geometry_msgs::msg::Point>("now_navigation_point"),  //导航点
                    BT::InputPort<geometry_msgs::msg::Point>("navigation_point"),  //巡逻路径给定点
                    BT::InputPort<geometry_msgs::msg::Point>("self_point"),
                    BT::OutputPort<geometry_msgs::msg::Point>("now_navigation_point")
                };
            }
            BT::NodeStatus tick() override;
            
    };
}

#endif