#include "node_library/patrol_1.hpp"

namespace BehaviorTree{

    Patrol1Node::Patrol1Node(const std::string& name, const BT::NodeConfig& config):
        BT::SyncActionNode(name,config){
            rclcpp::Time ti_now = rclcpp::Clock().now();
            num=0;
            clock=0;
            //初始化patrol_1_position，weight_enemy
            node_patrol1 = rclcpp::Node::make_shared("node_patrol1");                    
            patrol_radar_info = node_patrol1->create_subscription<robot_msgs::msg::AutoaimInfo>("radar_info",10,std::bind(&Patrol1Node::message_callback_patrol_radar_info,this,std::placeholders::_1));
            const auto map_file = toml::parse("config/battle_information.toml");
            map_position = toml::find<std::vector<std::vector<std::pair<double,double> > > >(map_file,"navigation_POS");

            for(auto vec:map_position){
                map_position_eigen.push_back(std::vector<Eigen::Vector3d>());
                auto it = map_position_eigen.end();
                for(auto data:vec){
                    it->push_back(Eigen::Vector3d(data.first,data.second,0));
                }
            }
        }

    void Patrol1Node::message_callback_patrol_radar_info(const robot_msgs::msg::AutoaimInfo &msg){
        robot_pos_array = msg.data;
        return;
    }

    BT::NodeStatus Patrol1Node::tick(){
        rclcpp::spin_some(node_patrol1);
        if(getInput<double>("time_begin").value()>rclcpp::Clock().now().seconds())return BT::NodeStatus::FAILURE;
        if(getInput<int>("situation").value()!=situation){
           new_situation=getInput<int>("situation").value();
           double num_distance=(patrol_1_position[num][0][situation]-patrol_1_position[0][0][new_situation])*(patrol_1_position[num][0][situation]-patrol_1_position[0][0][new_situation])
           +(patrol_1_position[num][1][situation]-patrol_1_position[0][1][new_situation])*(patrol_1_position[num][1][situation]-patrol_1_position[0][1][new_situation])
           +(patrol_1_position[num][2][situation]-patrol_1_position[0][2][new_situation])*(patrol_1_position[num][2][situation]-patrol_1_position[0][2][new_situation]);
            int min=0;
            for(int i=1;i<num_limit[new_situation];i++)
                if(num_distance>(patrol_1_position[num][0][situation]-patrol_1_position[i][0][new_situation])*(patrol_1_position[num][0][situation]-patrol_1_position[i][0][new_situation])
           +(patrol_1_position[num][1][situation]-patrol_1_position[i][1][new_situation])*(patrol_1_position[num][1][situation]-patrol_1_position[i][1][new_situation])
           +(patrol_1_position[num][2][situation]-patrol_1_position[i][2][new_situation])*(patrol_1_position[num][2][situation]-patrol_1_position[i][2][new_situation])){
            min=i;
            num_distance=(patrol_1_position[num][0][situation]-patrol_1_position[i][0][new_situation])*(patrol_1_position[num][0][situation]-patrol_1_position[i][0][new_situation])
           +(patrol_1_position[num][1][situation]-patrol_1_position[i][1][new_situation])*(patrol_1_position[num][1][situation]-patrol_1_position[i][1][new_situation])
           +(patrol_1_position[num][2][situation]-patrol_1_position[i][2][new_situation])*(patrol_1_position[num][2][situation]-patrol_1_position[i][2][new_situation]);
           }

             situation=new_situation;
             num=min;
        }
        int area_choose,enemy_num=0;
        clock=clock%clock_limit[situation];
        if(!getInput<int>("area_choose",area_choose)){
            return BT::NodeStatus::FAILURE;
        }
        geometry_msgs::msg::Point navigation_point;
        area_choose=getInput<int>("area_choose").value();
        size_t array_size = robot_pos_array.size();
        for(int i=0;i<array_size;i++){
            if(distance>(patrol_1_position[num][0][situation]-robot_pos_array[i].pos.x)*patrol_1_position[num][0][situation]-robot_pos_array[i].pos.x
            +(patrol_1_position[num][1][situation]-robot_pos_array[i].pos.y)*patrol_1_position[num][1][situation]-robot_pos_array[i].pos.y
            +(patrol_1_position[num][2][situation]-robot_pos_array[i].pos.z)*patrol_1_position[num][2][situation]-robot_pos_array[i].pos.z)
            enemy_num=+weight_enemy[robot_pos_array[i].id%100];
        }
        clock-=enemy_num;
        clock++;
        if(clock>clock_limit[situation]){
            num=(num+1)%num_limit[situation];
            setOutput<int>("area_choose",num);
        }
        navigation_point.x=patrol_1_position[num][0][situation];
        navigation_point.y=patrol_1_position[num][1][situation];
        navigation_point.z=patrol_1_position[num][2][situation];
        setOutput<geometry_msgs::msg::Point>("navigation_point",navigation_point);
        return BT::NodeStatus::SUCCESS;
        

}
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::Patrol1Node>("Patrol1Node");
// }