#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <toml.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "node_library/all_type_node.hpp"
#include "node_library/time_compu_interval.hpp"
#include "nav2_behavior_tree/plugins/control/pipeline_sequence.hpp"
#include <chrono>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a* member function as a callback from the timer. */


  static const char* xml_text = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="Untitled">
    <PipelineSequence>
      <BaseStaticAttackNode given_id="1" target_id="{attack_id}"/>
      <BaseAttackSpecificEnemyNode >
    </PipelineSequence>
  </BehaviorTree>
</root>
 )";


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);    

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<BehaviorTree::build_attack_node>("BuildAttackNode");
    factory.registerNodeType<BehaviorTree::ChassisAttackSpecificEnemyNode>("ChassisAttackSpecificEnemyNode");
    factory.registerNodeType<BehaviorTree::ChassisStaticAttackNode>("ChassisStaticAttackNode");
    factory.registerNodeType<BehaviorTree::ChassisDecideSpeedNode>("ChassisDecideSpeedNode");
    factory.registerNodeType<BehaviorTree::DecideEnemy>("DecideEnemy");
    factory.registerNodeType<BehaviorTree::DefenceBuildingNode>("DefenceBuildingNode");
    factory.registerNodeType<BehaviorTree::GimbalChooseEnemyNode>("GimbalChooseEnemyNode");
    factory.registerNodeType<BehaviorTree::lidar_attack_node>("LidarAttackNode");
    factory.registerNodeType<BehaviorTree::nav_heal_node>("NavHealNode");
    factory.registerNodeType<BehaviorTree::NavToEnemy>("NavToEnemy");
    factory.registerNodeType<BehaviorTree::NavToSpecificPlace>("NavToSpecificPlace");
    factory.registerNodeType<BehaviorTree::nav_u_node>("NavUNode");
    factory.registerNodeType<BehaviorTree::NavigationBuildingNode>("NavigationBuildingNode");
    factory.registerNodeType<BehaviorTree::OperatorGimbalNode>("OperatorGimbalNode");
    factory.registerNodeType<BehaviorTree::OperatorNavigateNode>("OperatorNavigateNode");
    factory.registerNodeType<BehaviorTree::Patrol1Node>("Patrol1Node");
    factory.registerNodeType<BehaviorTree::Patrol2Node>("Patrol2Node");
    factory.registerNodeType<BehaviorTree::Patrol3Node>("Patrol3Node");
    factory.registerNodeType<BehaviorTree::RadarDecision>("RadarDecision");
    factory.registerNodeType<BehaviorTree::Spin>("SpinNode");
    factory.registerNodeType<BehaviorTree::TimeBegin>("TimeBegin");
    factory.registerNodeType<BehaviorTree::TimeCompuLimit>("TimeCompuLimit");
    factory.registerNodeType<BehaviorTree::TimeCompuInterval>("TimeCompuInterval");
    factory.registerNodeType<BehaviorTree::TimeCtrl>("TimeCtrl");


    factory.registerNodeType<nav2_behavior_tree::PipelineSequence>("PipelineSequence");
    RCLCPP_INFO(rclcpp::get_logger("this is my logger"),"MYMYMY");
    auto tree = factory.createTreeFromFile(ROOT "config/tree2.xml");
    // auto nh=std::make_shared<Checkblood::SyncActionNode>();
    //rclcpp::spin(nh);
    while(rclcpp::ok()){     //rclcpp::ok()
       tree.tickWhileRunning();
    }
    rclcpp::shutdown();
    return 0;
}
