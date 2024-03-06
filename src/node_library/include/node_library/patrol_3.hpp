#ifndef RM_SENTRY_2024_PATROL_3_
#define RM_SENTRY_2024_PATROL_3_

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/point.h"
#include "robot_msgs/msg/autoaim_info.hpp"
#include "robot_msgs/msg/robot_info.hpp"
#include <toml.hpp>

namespace BehaviorTree{
    class Patrol3Node:public BT::SyncActionNode{
        private:
        public:
            rclcpp::Subscription<robot_msgs::msg::AutoaimInfo>::SharedPtr patrol_radar_info;
            rclcpp::Node::SharedPtr node_patrol1;
            Patrol3Node(const std::string&name, const BT::NodeConfig& config);
            int num,clock,situation,new_situation;
            int num_limit[5],clock_limit[5];
            std::vector< std::vector < std::pair < double,double > > > map_position;
            std::vector< std::vector < Eigen::Vector3d > > map_position_eigen ;
            double distance;
            double weight_enemy[9];  //0.8 0.8 0.8 0.8 0.8 0.8 0.8 0.8 0.8
            double time_limit;
            std::vector<robot_msgs::msg::RobotInfo> robot_pos_array;
            rclcpp::Time ti_now;
            void message_callback_patrol_radar_info(const robot_msgs::msg::AutoaimInfo &msg);
            static BT::PortsList providedPorts(){
                return {
                    BT::BidirectionalPort<int>("area_choose"),
                    BT::InputPort<double>("time_begin"),
                    BT::OutputPort<geometry_msgs::msg::Point>("navigation_point")  ////目标点的坐标
                };
            }
            BT::NodeStatus tick() override;
            
    };
}

#endif