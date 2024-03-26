#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"

#include "gripper_behaviors.cpp"
#include "camera_behaviors.cpp"
#include "arm_behaviors.cpp"
#include "database_behaviors.cpp"
#include "behaviortree_ros2/plugins.hpp"
#include "yaml-cpp/yaml.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "iostream"

#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/loggers/bt_sqlite_logger.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/json_export.h"

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

const std::string default_bt_xml_foler = 
    ament_index_cpp::get_package_share_directory("behavior_tree_ros2_rob8") + "/bt_xml";
  // Simple tree, used to execute once each action.
  static const char* xml_text = R"(
 <root BTCPP_format="4">
     <BehaviorTree ID="test_xdl">
        <Sequence>
            <PrintValue message="start"/>
            <SubTree ID="Add" reagent='carbonyl' vessel='reactor' amount='2 eq'/>
            <SubTree ID="Add"/>
            <SubTree ID="Add"/>
        </Sequence>
     </BehaviorTree>
 </root>
 )";

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("test_client");

  BehaviorTreeFactory factory;

  factory.registerNodeType<PrintValue>("PrintValue");

    //////////////////DATABASE/////////////////
  RosNodeParams params_database_add_chemical;
  params_database_add_chemical.nh = nh;
  params_database_add_chemical.server_timeout = std::chrono::milliseconds(2000);
  params_database_add_chemical.wait_for_server_timeout = std::chrono::milliseconds(1000);
  params_database_add_chemical.default_port_value = "add_chemical_service";
  factory.registerNodeType<AddChemicalNode>("AddChemicalNode",params_database_add_chemical);

  RosNodeParams params_database_add_workstation;
  params_database_add_workstation.nh = nh;
  params_database_add_workstation.server_timeout = std::chrono::milliseconds(2000);
  params_database_add_workstation.wait_for_server_timeout = std::chrono::milliseconds(1000);
  params_database_add_workstation.default_port_value = "add_workstation_service";
  factory.registerNodeType<AddWorkstationNode>("AddWorkstationNode",params_database_add_workstation);

  RosNodeParams params_database_upsert_chemical_location;
  params_database_upsert_chemical_location.nh = nh;
  params_database_upsert_chemical_location.server_timeout = std::chrono::milliseconds(2000);
  params_database_upsert_chemical_location.wait_for_server_timeout = std::chrono::milliseconds(1000);
  params_database_upsert_chemical_location.default_port_value = "upsert_chemical_location_service";
  factory.registerNodeType<UpsertChemicalLocationNode>("UpsertChemicalLocationNode",params_database_upsert_chemical_location);

  RosNodeParams params_database_upsert_workstation_location;
  params_database_upsert_workstation_location.nh = nh;
  params_database_upsert_workstation_location.server_timeout = std::chrono::milliseconds(2000);
  params_database_upsert_workstation_location.wait_for_server_timeout = std::chrono::milliseconds(1000);
  params_database_upsert_workstation_location.default_port_value = "upsert_workstation_location_service";
  factory.registerNodeType<UpsertWorkstationLocationNode>("UpsertWorkstationLocationNode",params_database_upsert_workstation_location);

  RosNodeParams params_database_get_all_chemical_locations;
  params_database_get_all_chemical_locations.nh = nh;
  params_database_get_all_chemical_locations.server_timeout = std::chrono::milliseconds(2000);
  params_database_get_all_chemical_locations.wait_for_server_timeout = std::chrono::milliseconds(1000);
  params_database_get_all_chemical_locations.default_port_value = "get_all_chemical_locations_service";
  factory.registerNodeType<GetAllChemicalLocationsNode>("GetAllChemicalLocationsNode",params_database_get_all_chemical_locations);

  RosNodeParams params_database_get_all_workstaion_locations;
  params_database_get_all_workstaion_locations.nh = nh;
  params_database_get_all_workstaion_locations.server_timeout = std::chrono::milliseconds(2000);
  params_database_get_all_workstaion_locations.wait_for_server_timeout = std::chrono::milliseconds(1000);
  params_database_get_all_workstaion_locations.default_port_value = "get_all_workstation_locations_service";
  factory.registerNodeType<GetAllWorkstationLocationsNode>("GetAllWorkstationLocationsNode",params_database_get_all_workstaion_locations);

  RosNodeParams params_database_remove_chemical;
  params_database_remove_chemical.nh = nh;
  params_database_remove_chemical.server_timeout = std::chrono::milliseconds(2000);
  params_database_remove_chemical.wait_for_server_timeout = std::chrono::milliseconds(1000);
  params_database_remove_chemical.default_port_value = "remove_chemical_service";
  factory.registerNodeType<RemoveChemicalNode>("RemoveChemicalNode",params_database_remove_chemical);

  RosNodeParams params_database_remove_workstation;
  params_database_remove_workstation.nh = nh;
  params_database_remove_workstation.server_timeout = std::chrono::milliseconds(2000);
  params_database_remove_workstation.wait_for_server_timeout = std::chrono::milliseconds(1000);
  params_database_remove_workstation.default_port_value = "remove_workstation_service";
  factory.registerNodeType<RemoveWorkstationNode>("RemoveWorkstationNode",params_database_remove_workstation);

  ///////////////////////////////////


  //////////////////////////////////////////////////////////////////////////////
  RosNodeParams params_gripper_franka_grasp;
  params_gripper_franka_grasp.nh = nh;
  params_gripper_franka_grasp.server_timeout = std::chrono::milliseconds(2000);
  params_gripper_franka_grasp.wait_for_server_timeout = std::chrono::milliseconds(1000);
  params_gripper_franka_grasp.default_port_value = "panda_gripper/grasp";
  factory.registerNodeType<FrankaGraspGripperAction>("FrankaGraspGripperAction",params_gripper_franka_grasp);
  //////////////////////////////////////////////////////////////////////////////
  RosNodeParams params_gripper_franka_homing;
  params_gripper_franka_homing.nh = nh;
  params_gripper_franka_homing.server_timeout = std::chrono::milliseconds(2000);
  params_gripper_franka_homing.wait_for_server_timeout = std::chrono::milliseconds(1000);
  params_gripper_franka_homing.default_port_value = "panda_gripper/homing";
  factory.registerNodeType<FrankaHomeGripperAction>("FrankaHomeGripperAction",params_gripper_franka_homing);
  ////////////////////////////////////////////////////////////////////////////
  RosNodeParams params_gripper_franka_move;
  params_gripper_franka_move.nh = nh;
  params_gripper_franka_move.server_timeout = std::chrono::milliseconds(2000);
  params_gripper_franka_move.wait_for_server_timeout = std::chrono::milliseconds(1000);
  params_gripper_franka_move.default_port_value = "panda_gripper/move";
  factory.registerNodeType<FrankaMoveGripperAction>("FrankaMoveGripperAction",params_gripper_franka_move);
  ////////////////////////////////////////////////////////////////////////////////////////
  
  RosNodeParams params_gripper;
  params_gripper.nh = nh;
  params_gripper.server_timeout = std::chrono::milliseconds(2000);
  params_gripper.wait_for_server_timeout = std::chrono::milliseconds(1000);
  params_gripper.default_port_value = "gripper_service";
  factory.registerNodeType<GripperAction>("GripperAction",params_gripper);
////////////////////////////////////////////////////////////////////////////////////////
  RosNodeParams params_aruco;
  params_aruco.nh = nh;
  params_aruco.server_timeout = std::chrono::milliseconds(2000);
  params_aruco.wait_for_server_timeout = std::chrono::milliseconds(1000);
  params_aruco.default_port_value = "detect_marker_pose";
  factory.registerNodeType<FindArucoTagAction>("ArucoAction",params_aruco);
////////////////////////////////////////////////////////////////////////////////////////
  RosNodeParams params_arm_mode_pose;
  params_arm_mode_pose.nh = nh;
  params_arm_mode_pose.server_timeout = std::chrono::milliseconds(2000);
  params_arm_mode_pose.wait_for_server_timeout = std::chrono::milliseconds(1000);
  params_arm_mode_pose.default_port_value = "arm_move_pose_service";
  factory.registerNodeType<ArmMovePoseAction>("ArmMovePoseAction",params_arm_mode_pose);

  RosNodeParams params_arm_mode_joints;
  params_arm_mode_joints.nh = nh;
  params_arm_mode_joints.server_timeout = std::chrono::milliseconds(2000);
  params_arm_mode_joints.wait_for_server_timeout = std::chrono::milliseconds(1000);
  params_arm_mode_joints.default_port_value = "arm_move_joints_service";
  factory.registerNodeType<ArmMoveJointsAction>("ArmMoveJointsAction",params_arm_mode_joints);

  RosNodeParams params_gripper_joint;
  params_gripper_joint.nh = nh;

  params_gripper_joint.default_port_value = "gripper_joint_service";
  factory.registerNodeType<GripperJointAction>("GripperJointAction",params_gripper_joint);
  
  factory.registerNodeType<ArmArrayToPoseAction>("ArmArrayToPoseAction");
  factory.registerNodeType<ArmPoseMsgOffsetCalculation>("ArmPoseMsgOffsetCalculation");

  RosNodeParams params_arm_move_pose_msg;
  params_arm_move_pose_msg.nh = nh;
  params_arm_move_pose_msg.default_port_value = "arm_move_pose_msg_service";
  params_arm_move_pose_msg.server_timeout = std::chrono::milliseconds(2000);
  params_arm_move_pose_msg.wait_for_server_timeout = std::chrono::milliseconds(1000);
  factory.registerNodeType<ArmMovePoseMsgAction>("ArmMovePoseMsgAction",params_arm_move_pose_msg);

  RosNodeParams params_arm_move_pliz_ptp_pose_msg;
  params_arm_move_pliz_ptp_pose_msg.nh = nh;
  params_arm_move_pliz_ptp_pose_msg.default_port_value = "arm_move_pliz_ptp_pose_msg_service";
  params_arm_move_pliz_ptp_pose_msg.server_timeout = std::chrono::milliseconds(2000);
  params_arm_move_pliz_ptp_pose_msg.wait_for_server_timeout = std::chrono::milliseconds(1000);
  factory.registerNodeType<ArmMovePlizPtpPoseMsgAction>("ArmMovePlizPtpPoseMsgAction",params_arm_move_pliz_ptp_pose_msg);


  RosNodeParams params_arm_move_pliz_lin_pose_msg;
  params_arm_move_pliz_lin_pose_msg.nh = nh;
  params_arm_move_pliz_lin_pose_msg.default_port_value = "arm_move_pliz_lin_pose_msg_service";
  params_arm_move_pliz_lin_pose_msg.server_timeout = std::chrono::milliseconds(2000);
  params_arm_move_pliz_lin_pose_msg.wait_for_server_timeout = std::chrono::milliseconds(1000);
  factory.registerNodeType<ArmMovePlizLinPoseMsgAction>("ArmMovePlizLinPoseMsgAction",params_arm_move_pliz_lin_pose_msg);

  RosNodeParams params_home_arm;
  params_home_arm.nh = nh;
  params_home_arm.server_timeout = std::chrono::milliseconds(2000);
  params_home_arm.wait_for_server_timeout = std::chrono::milliseconds(1000);
  params_home_arm.default_port_value = "home_arm";
  factory.registerNodeType<HomeArmAction>("HomeArmAction",params_home_arm);

  RosNodeParams params;
  params.nh = nh;
  params.server_timeout = std::chrono::milliseconds(2000);
  params.wait_for_server_timeout = std::chrono::milliseconds(1000);
  params.default_port_value = "sleep_service";

  factory.registerNodeType<ArmPoseOffsetCalculation>("ArmPoseOffsetCalculation");

#ifdef USE_SLEEP_PLUGIN
  RegisterRosNode(factory, "../lib/libsleep_action_plugin.so", params);
#else
  factory.registerNodeType<SleepAction>("SleepAction", params);
#endif

  // nh->declare_parameter<std::string>("tree_xml_file", default_bt_xml_file);
  // std::string tree_xml_file_ = nh->get_parameter("tree_xml_file").as_string();
    // std::string search_directory = "./";
  
  // using std::filesystem::directory_iterator;
  // for (auto const& entry : directory_iterator(default_bt_xml_foler)) 
  // {
  //   if( entry.path().extension() == ".xml")
  //   {
  //     factory.registerBehaviorTreeFromFile(entry.path().string());
  //   }
  // }
  

  factory.registerBehaviorTreeFromFile(ament_index_cpp::get_package_share_directory("behavior_tree_ros2_rob8") + "/bt_xml/panda_test.xml");
  factory.registerBehaviorTreeFromFile(ament_index_cpp::get_package_share_directory("behavior_tree_ros2_rob8") + "/bt_xml/stirring.xml");
  factory.registerBehaviorTreeFromFile(ament_index_cpp::get_package_share_directory("behavior_tree_ros2_rob8") + "/bt_xml/mobile_base_trees.xml");
  factory.registerBehaviorTreeFromFile(ament_index_cpp::get_package_share_directory("behavior_tree_ros2_rob8") + "/bt_xml/manipulator_trees.xml");
  factory.registerBehaviorTreeFromFile(ament_index_cpp::get_package_share_directory("behavior_tree_ros2_rob8") + "/bt_xml/liquid_handling.xml");
  factory.registerBehaviorTreeFromFile(ament_index_cpp::get_package_share_directory("behavior_tree_ros2_rob8") + "/bt_xml/database_trees.xml");
  std::string xml_models = BT::writeTreeNodesModelXML(factory);
  std::cout << "----------- XML file  ----------\n"
            << xml_models
            << "--------------------------------\n";

  // factory.registerBehaviorTreeFromFile(tree_xml_file_);
  auto tree = factory.createTree("MainTree");

  // std::cout << BT::writeTreeToXML(tree);
  std::cout << "----------- XML file  ----------\n"
            << BT::WriteTreeToXML(tree, false, false)
            << "--------------------------------\n";

  BT::Groot2Publisher publisher(tree,5555);

  // auto tree = factory.createTreeFromText(xml_text);
//   tree.tickWhileRunning();
  // tree.tickOnce();
  // for(int i=0; i<5; i++){
  //   tree.tickWhileRunning();
  //   // tree.tickOnce();
  // }

    // let's visualize some information about the current state of the blackboards.
  std::cout << "\n------ First BB ------" << std::endl;
  tree.subtrees[0]->blackboard->debugMessage();
  std::cout << "\n------ Second BB------" << std::endl;
  tree.subtrees[1]->blackboard->debugMessage();

  return 0;
}
