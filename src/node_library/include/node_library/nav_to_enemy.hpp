#ifndef RM_SENTRY_2024_NAV_TO_ENEMY_
#define RM_SENTRY_2024_NAV_TO_ENEMY_


#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include"geometry_msgs/msg/point.h"
//#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "robot_msgs/msg/autoaim_info.hpp"
#include "robot_msgs/msg/walk_cmd.hpp"


namespace BehaviorTree{
    class NavToEnemy:public BT::SyncActionNode{
        private:
            bool if_aviod = false;
        public:
            NavToEnemy(const std::string&name, const BT::NodeConfig& config);
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr my_pos;
            rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr decision_pos;
            rclcpp::Subscription<robot_msgs::msg::AutoaimInfo>::SharedPtr subscription_enemy_pos;
            rclcpp::Node::SharedPtr node1;
            rclcpp::Node::SharedPtr node2;
            rclcpp::Node::SharedPtr node3;
            std::vector<robot_msgs::msg::RobotInfo> robot_pos_array;
            geometry_msgs::msg::Point enemy_pos_pub;
            geometry_msgs::msg::Point enemy_pos_deal;
            geometry_msgs::msg::Point My_pos;
            void avoid_fight(geometry_msgs::msg::Point Point,geometry_msgs::msg::Point Point_start,geometry_msgs::msg::Point Point_my,geometry_msgs::msg::Point &Point_pub,double d);
            void message_callback_enemy_pos(const robot_msgs::msg::AutoaimInfo &msg);
            void message_callback_my_pos(const nav_msgs::msg::Odometry &msg);
            static BT::PortsList providedPorts(){
                return {
                    BT::InputPort<int>("now_enemy_id")
                };
            }
            BT::NodeStatus tick() override;
            
    };
}

#endif