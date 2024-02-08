#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <toml.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "node_library/all_type_node.hpp"
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
   factory.registerNodeType<BehaviorTree::BaseStaticAttackNode>("BaseStaticAttackNode");
   factory.registerNodeType<BehaviorTree::BaseAttackSpecificEnemyNode>("BaseAttackSpecificEnemyNode");
   factory.registerNodeType<BehaviorTree::DefenceBuildingNode>("DefenceBuildingNode");
   factory.registerNodeType<BehaviorTree::GimbalChooseEnemyNode>("GimbalChooseEnemyNode");
   factory.registerNodeType<BehaviorTree::NavigationBuildingNode>("NavigationBuildingNode");
   factory.registerNodeType<BehaviorTree::DecideEnemy>("DecideEnemy");
   factory.registerNodeType<BehaviorTree::NavToEnemy>("NavToEnemy");
   factory.registerNodeType<BehaviorTree::PATROL1Node>("PATROL1Node");
   factory.registerNodeType<BehaviorTree::PATROL2Node>("PATROL2Node");
   factory.registerNodeType<BehaviorTree::RadarIndependent>("RadarIndependent");
   factory.registerNodeType<BehaviorTree::TimeBegin>("TimeBegin");
   factory.registerNodeType<BehaviorTree::TimeCtrl>("TimeCtrl");
   factory.registerNodeType<BehaviorTree::TimeCompu>("TimeCompu");
   factory.registerNodeType<BehaviorTree::Spin>("SpinNode");


   factory.registerNodeType<nav2_behavior_tree::PipelineSequence>("PipelineSequence");
   RCLCPP_INFO(rclcpp::get_logger("this is my logger"),"MYMYMY");
   auto tree = factory.createTreeFromFile(ROOT "config/tree.xml");
   // auto nh=std::make_shared<Checkblood::SyncActionNode>();
    //rclcpp::spin(nh);
    while(rclcpp::ok()){     //rclcpp::ok()
       tree.tickWhileRunning();
    }
    rclcpp::shutdown();
    return 0;
}
