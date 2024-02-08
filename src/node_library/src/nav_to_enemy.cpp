#include "node_library/nav_to_enemy.hpp"

namespace BehaviorTree{

    NavToEnemy::NavToEnemy(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                    node1 = rclcpp::Node::make_shared("subscriber_enemy_pos");   
                    node2 = rclcpp::Node::make_shared("decision_pos");  
                    node3 = rclcpp::Node::make_shared("Odometry");                
                    subscription_enemy_pos = node1->create_subscription<robot_msgs::msg::AutoaimInfo>("autoaim2decision",10,std::bind(&NavToEnemy::message_callback_enemy_pos,this,std::placeholders::_1));
                    my_pos = node3->create_subscription<nav_msgs::msg::Odometry>("Odometry",10,std::bind(&NavToEnemy::message_callback_my_pos,this,std::placeholders::_1));
                    decision_pos = node2->create_publisher<geometry_msgs::msg::Point>("decision2nav_pos", 10);
                }

    void NavToEnemy::message_callback_enemy_pos(const robot_msgs::msg::AutoaimInfo &msg){
        robot_pos_array = msg.data;
        return;
    }
    
    void NavToEnemy::message_callback_my_pos(const nav_msgs::msg::Odometry &msg){
        //Header header
        //string child_frame_id
       // geometry_msgs/TwistWithCovariance twist
        My_pos = msg.pose.pose.position;
        return;
    }

    void NavToEnemy::avoid_fight(geometry_msgs::msg::Point Point,geometry_msgs::msg::Point Point_start,geometry_msgs::msg::Point Point_my,geometry_msgs::msg::Point &Point_pub,double d){
       //障碍信息,高度
       static geometry_msgs::msg::Point pos_decision;
       double x1,x2,y1,y2,a;
       a = atan((Point.x-Point_start.x)/(Point_start.y-Point.y));
       x1 = Point_start.x+d*cos(a);
       x2 = Point_start.x-d*cos(a);
       y1 = Point_start.y+d*sin(a);
       y2 = Point_start.y-d*sin(a);
      if(sqrt((Point_my.x-x1)*(Point_my.x-x1)+(Point_my.y-y1)*(Point_my.y-y1))<1){
        pos_decision.x = Point_start.x;
        pos_decision.y = Point_start.y;
      }
      else if(sqrt((Point_my.x-Point_start.x)*(Point_my.x-Point_start.x)+(Point_my.y-Point_start.y)*(Point_my.y-Point_start.y))<1){
        pos_decision.x = x1;
        pos_decision.y = y1;
      }
      Point_pub = pos_decision;
       
}

    BT::NodeStatus NavToEnemy::tick(){
        static geometry_msgs::msg::Point start_avoid_point;
        RCLCPP_INFO(rclcpp::get_logger("move_to_enemy_node"),"I'm ticked");
        rclcpp::spin_some(node1);
        rclcpp::spin_some(node2);
        rclcpp::spin_some(node3);
        size_t array_size = robot_pos_array.size();
        int now_enemy_id;
        if(!getInput<int>("now_enemy_id",now_enemy_id)){
            RCLCPP_INFO(rclcpp::get_logger("move_to_enemy_node"),"No Response");
            return BT::NodeStatus::FAILURE;
        }

        for(int i=0;i<array_size;++i){
            if(robot_pos_array[i].id == now_enemy_id){
                enemy_pos_deal.x=robot_pos_array[i].pos.x;
                enemy_pos_deal.y=robot_pos_array[i].pos.y;
                enemy_pos_deal.z=robot_pos_array[i].pos.z;
            }
        }
    double distance = sqrt((enemy_pos_deal.x-My_pos.x)*(enemy_pos_deal.x-My_pos.x)+(enemy_pos_deal.y-My_pos.y)*(enemy_pos_deal.y-My_pos.y)+(enemy_pos_deal.z-My_pos.z)*(enemy_pos_deal.z-My_pos.z));
       if(distance > 2&& !if_aviod/*？*/){
        enemy_pos_pub = enemy_pos_deal;
        start_avoid_point = My_pos;
       }
       else if(distance <= 2&&if_aviod == false){
        if_aviod = true;
       }
       else{
        /*bug*/
        NavToEnemy::avoid_fight(enemy_pos_deal,start_avoid_point,My_pos,enemy_pos_pub,distance);
    }
    decision_pos->publish(enemy_pos_pub);
    return BT::NodeStatus::SUCCESS;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::NavToEnemy>("NavToEnemy");
// }