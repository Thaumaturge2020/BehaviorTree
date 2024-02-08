#include "node_library/decide_enemy.hpp"

namespace BehaviorTree{

    DecideEnemy::DecideEnemy(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                    node1 = rclcpp::Node::make_shared("subscriber_enemy_pos");   
                    node2 = rclcpp::Node::make_shared("subscriber_enemy_blood"); 
                    node3 = rclcpp::Node::make_shared("Odometry");                  
                    subscription_enemy_pos = node1->create_subscription<robot_msgs::msg::AutoaimInfo>("autoaim2decision",10,std::bind(&DecideEnemy::message_callback_enemy_pos,this,std::placeholders::_1));
                    subscription_enemy_blood = node2->create_subscription<robot_msgs::msg::RobotBloodInfo>("enemy_info",10,std::bind(&DecideEnemy::message_callback_enemy_blood,this,std::placeholders::_1));
                    my_pos = node3->create_subscription<nav_msgs::msg::Odometry>("Odometry",10,std::bind(&DecideEnemy::message_callback_my_pos,this,std::placeholders::_1));
                }
                

    void DecideEnemy::message_callback_enemy_pos(const robot_msgs::msg::AutoaimInfo &msg){
        robot_pos_array = msg.data;
        return;
    }

    void DecideEnemy::message_callback_enemy_blood(const robot_msgs::msg::RobotBloodInfo &msg){
        robot_blood_array = msg.data;
        return;
    }

    void DecideEnemy::message_callback_my_pos(const nav_msgs::msg::Odometry &msg){
        //Header header
        //string child_frame_id
       // geometry_msgs/TwistWithCovariance twist
        My_pos = msg.pose.pose.position;
        return;
    }
    
    double Distance(double x1,double y1,double x2,double y2){
        return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1+y2));
    }
    
     double Critical(int blood,double distance){
        return 0.5*blood/300.0+0.5*distance/5;//(改)
     }

    BT::NodeStatus DecideEnemy::tick(){
        RCLCPP_INFO(rclcpp::get_logger("decide_enemy_node"),"I'm ticked");
        rclcpp::spin_some(node1);
        rclcpp::spin_some(node2);
        rclcpp::spin_some(node3);
        size_t array_size = robot_pos_array.size();
        int now_enemy_id;
        if(!getInput<int>("now_enemy_id",now_enemy_id)){
            RCLCPP_INFO(rclcpp::get_logger("decide_enemy_node"),"No Response");
            return BT::NodeStatus::FAILURE;
        }
        int Id,Blood;
        Blood = robot_blood_array[0].blood;
        Id = robot_blood_array[0].id;
        //距离单位
        double distance = Distance(My_pos.x,My_pos.y,robot_pos_array[0].pos.x,robot_pos_array[0].pos.y);
        double mark = Critical(robot_blood_array[0].blood,distance);
        for(int i=1;i<array_size;++i){
            double a = Distance(My_pos.x,My_pos.y,robot_pos_array[i].pos.x,robot_pos_array[i].pos.y);
            double b = Critical(robot_blood_array[i].blood,a);
            if(b<mark){
                Blood = robot_blood_array[i].blood;
                Id = robot_blood_array[i].id;
                mark = b;
                distance = a;
            }
        }
        setOutput<int>("target_enemy_id",Id);
        return BT::NodeStatus::SUCCESS;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::DecideEnemy>("DecideEnemy");
// }
