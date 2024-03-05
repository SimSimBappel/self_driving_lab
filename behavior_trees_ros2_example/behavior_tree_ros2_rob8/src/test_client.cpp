#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"
#include "gripper_behaviors.cpp"
#include "arm_behaviors.cpp"
#include "behaviortree_ros2/plugins.hpp"
#include "yaml-cpp/yaml.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

#ifndef USE_SLEEP_PLUGIN
#include "sleep_action.hpp"
#endif

using namespace BT;

//-------------------------------------------------------------
// Simple Action to print a number
//-------------------------------------------------------------

class PrintValue : public BT::SyncActionNode
{
public:
  PrintValue(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config) {}

  BT::NodeStatus tick() override {
    std::string msg;
    if( getInput("message", msg ) ){
      std::cout << "PrintValue: " << msg << std::endl;
      return NodeStatus::SUCCESS;
    }
    else{
      std::cout << "PrintValue FAILED "<< std::endl;
      return NodeStatus::FAILURE;
    }
  }

  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("message") };
  }
};




//-----------------------------------------------------


const std::string default_bt_xml_file = 
    ament_index_cpp::get_package_share_directory("behavior_tree_ros2_rob8") + "/bt_xml/panda_test.xml";

  // Simple tree, used to execute once each action.
//   static const char* xml_text = R"(
//  <root BTCPP_format="4">
//      <BehaviorTree>
//         <Sequence>
//             <PrintValue message="start"/>
//             <GripperAction name="gripper_open" open="true"/>
//             <ArmMovePoseAction name="arm" pose="0.45,0.1,0.165,0.0,1.529,0.0"/>
//             <ArmMovePoseAction name="arm" pose="0.45,0.1,0.1,0.0,1.529,0.0"/>
//             <GripperAction name="gripper_close" open="false"/>
//             <ArmMovePoseAction name="arm" pose="0.45,0.1,0.165,0.0,1.529,0.0"/>
//             <ArmMovePoseAction name="arm" pose="-0.1,-0.6,0.4,0.0,0.0,-1.529"/>
//             <ArmMovePoseAction name="arm" pose="0.45,0.1,0.165,0.0,1.529,0.0"/>
//             <ArmMovePoseAction name="arm" pose="0.45,0.1,0.1,0.0,1.529,0.0"/>
//             <GripperAction name="gripper_open" open="true"/>
//             <ArmMovePoseAction name="arm" pose="0.45,0.1,0.165,0.0,1.529,0.0"/>
//             <ArmMovePoseAction name="arm" pose="0.1,-0.6,0.4,0.0,0.0,-1.529"/>
//             <GripperAction name="gripper_close" open="false"/>
//             <PrintValue message="sleep completed"/>
//         </Sequence>
//      </BehaviorTree>
//  </root>
//  )";

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("test_client");

  BehaviorTreeFactory factory;

  factory.registerNodeType<PrintValue>("PrintValue");
  
  RosNodeParams params_gripper;
  params_gripper.nh = nh;
  params_gripper.server_timeout = std::chrono::milliseconds(2000);
  params_gripper.wait_for_server_timeout = std::chrono::milliseconds(1000);
  params_gripper.default_port_value = "gripper_service";
  factory.registerNodeType<GripperAction>("GripperAction",params_gripper);

  RosNodeParams params_arm_mode_pose;
  params_arm_mode_pose.nh = nh;
  params_arm_mode_pose.server_timeout = std::chrono::milliseconds(2000);
  params_arm_mode_pose.wait_for_server_timeout = std::chrono::milliseconds(1000);
  params_arm_mode_pose.default_port_value = "arm_move_pose_service";
  factory.registerNodeType<ArmMovePoseAction>("ArmMovePoseAction",params_arm_mode_pose);

  RosNodeParams params_gripper_joint;
  params_gripper_joint.nh = nh;

  params_gripper_joint.default_port_value = "gripper_joint_service";
  factory.registerNodeType<GripperJointAction>("GripperJointAction",params_gripper_joint);

  RosNodeParams params;
  params.nh = nh;
  params.server_timeout = std::chrono::milliseconds(2000);
  params.wait_for_server_timeout = std::chrono::milliseconds(1000);
  params.default_port_value = "sleep_service";

#ifdef USE_SLEEP_PLUGIN
  RegisterRosNode(factory, "../lib/libsleep_action_plugin.so", params);
#else
  factory.registerNodeType<SleepAction>("SleepAction", params);
#endif

  nh->declare_parameter<std::string>("tree_xml_file", default_bt_xml_file);
  std::string tree_xml_file_ = nh->get_parameter("tree_xml_file").as_string();
  auto tree = factory.createTreeFromFile(tree_xml_file_);
  // auto tree = factory.createTreeFromText(xml_text);
  // tree.tickWhileRunning();
  // tree.tickOnce();
  for(int i=0; i<5; i++){
    tree.tickWhileRunning();
    // tree.tickOnce();
  }

  return 0;
}
