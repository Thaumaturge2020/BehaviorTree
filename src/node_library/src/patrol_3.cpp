#include "node_library/patrol_3.hpp"

namespace BehaviorTree{

    Patrol3Node::Patrol3Node(const std::string& name, const BT::NodeConfig& config):
        BT::SyncActionNode(name,config){
            RCLCPP_INFO(rclcpp::get_logger("patrol_3_node"),"Patrol3Node initialized");

            rclcpp::Time ti_now = rclcpp::Clock().now();
            num=0;
            clock=0;
            situation = 0;
            //初始化patrol_1_position，weight_enemy
            node_patrol1 = rclcpp::Node::make_shared("node_patrol1");                    
            patrol_radar_info = node_patrol1->create_subscription<robot_msgs::msg::AutoaimInfo>("radar_info",10,std::bind(&Patrol3Node::message_callback_patrol_radar_info,this,std::placeholders::_1));
            const auto map_file = toml::parse(ROOT "config/battle_information.toml");
            map_position = toml::find<std::vector<std::vector<std::pair<double,double> > > >(map_file,"navigation_POS");
            time_limit = toml::find<double>(map_file,"time_limit");

            for(auto vec:map_position){
                map_position_eigen.push_back(std::vector<Eigen::Vector3d>());
                auto it = map_position_eigen.end();
                --it;
                for(auto data:vec){
                    it->push_back(Eigen::Vector3d(data.first,data.second,0));
                }
            }

            RCLCPP_INFO(rclcpp::get_logger("patrol_3_node"),"Patrol3Node initialized");
        }

    void Patrol3Node::message_callback_patrol_radar_info(const robot_msgs::msg::AutoaimInfo &msg){
        robot_pos_array = msg.data;
        return;
    }

    BT::NodeStatus Patrol3Node::tick(){
        rclcpp::spin_some(node_patrol1);
        // RCLCPP_INFO(rclcpp::get_logger("patrol_3_node"),"%lf",(rclcpp::Clock().now() - ti_now).seconds());
        if((rclcpp::Clock().now() - ti_now).seconds() < time_limit) return BT::NodeStatus::SUCCESS;
        ti_now = rclcpp::Clock().now();
        int area_choose,enemy_num=0,new_situation;
        RCLCPP_INFO(rclcpp::get_logger("patrol_3_node"),"FAILED");
        if(!getInput<int>("area_choose",area_choose)){
            RCLCPP_INFO(rclcpp::get_logger("patrol_3_node"),"FAILED");
            return BT::NodeStatus::FAILURE;
        }
        else{
            RCLCPP_INFO(rclcpp::get_logger("patrol_3_node"),"%d",area_choose);
            if(area_choose >= map_position_eigen.size()) return BT::NodeStatus::FAILURE;
            new_situation = situation + 1;
            new_situation = map_position_eigen[area_choose].size() ? 0 : new_situation;
        }
        

        geometry_msgs::msg::Point navigation_point;
        area_choose=getInput<int>("area_choose").value();
        navigation_point.x = map_position_eigen[area_choose][new_situation].x();
        navigation_point.y = map_position_eigen[area_choose][new_situation].y();
        navigation_point.z = map_position_eigen[area_choose][new_situation].z();
        setOutput<geometry_msgs::msg::Point>("navigation_point",navigation_point);
        RCLCPP_INFO(rclcpp::get_logger("patrol_3_node"),"%lf %lf %lf",navigation_point.x,navigation_point.y,navigation_point.z);
        return BT::NodeStatus::SUCCESS;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::Patrol3Node>("Patrol3Node");
// }