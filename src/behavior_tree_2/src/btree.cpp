#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "node_library/base_static_attack_node.hpp"
#include "nav2_behavior_tree/plugins/control/pipeline_sequence.hpp"
#include <chrono>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a* member function as a callback from the timer. */


  static const char* xml_text = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="Untitled">
    <PipelineSequence>
      <BaseStaticAttackNode given_id="1" target_id="{attack_id}"/>
    </PipelineSequence>
  </BehaviorTree>
</root>
 )";


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);    

    BT::BehaviorTreeFactory factory;
   factory.registerNodeType<BehaviorTree::BaseStaticAttackNode>("BaseStaticAttackNode");
   factory.registerNodeType<nav2_behavior_tree::PipelineSequence>("PipelineSequence");
   auto tree = factory.createTreeFromText(xml_text);
   // auto nh=std::make_shared<Checkblood::SyncActionNode>();
    //rclcpp::spin(nh);
    while(rclcpp::ok()){     //rclcpp::ok()
       tree.tickWhileRunning();
    }
    rclcpp::shutdown();
    return 0;
}
