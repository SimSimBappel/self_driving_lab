#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"

#include "gripper_behaviors.cpp"
#include "camera_behaviors.cpp"
#include "arm_behaviors.cpp"
#include "database_behaviors.cpp"
#include "mir_behaviors.cpp"
#include "behaviortree_ros2/plugins.hpp"
#include "yaml-cpp/yaml.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "iostream"
#include <memory>

#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/loggers/bt_sqlite_logger.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/json_export.h"

#include <chrono>
#include <random>

#include "behavior_tree_ros2_actions/srv/xdl.hpp"

#ifndef USE_SLEEP_PLUGIN
#include "sleep_action.hpp"
#endif

using namespace BT;
using namespace std::chrono_literals;

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


class BehaviorServer : public rclcpp::Node 
{
  public:
    BehaviorServer(const rclcpp::NodeOptions &options)
    : Node("behavior_services_node", options), node_(std::make_shared<rclcpp::Node>("behavior_services_node"))
          ,executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
    {
    // BehaviorServer()
    // : Node("behavior_services_node")
    // {
    
      this->service_ = create_service<behavior_tree_ros2_actions::srv::Xdl>("xdl_service", std::bind(&BehaviorServer::Xdl_service, this,
                                std::placeholders::_1, std::placeholders::_2));
    

        

        factory.registerNodeType<PrintValue>("PrintValue");
      RosNodeParams params_aruco_lookup_transform;
  params_aruco_lookup_transform.nh = this->node_;
  params_aruco_lookup_transform.server_timeout = std::chrono::milliseconds(2000);
  params_aruco_lookup_transform.wait_for_server_timeout = std::chrono::milliseconds(1000);
  params_aruco_lookup_transform.default_port_value = "lookup_transform";
  factory.registerNodeType<LookupTransformNode>("LookupTransformNode",params_aruco_lookup_transform);

        RosNodeParams params_database_get_chemicals;
  // params_database_get_chemicals.nh = shared_from_this();
  params_database_get_chemicals.nh = this->node_;
  params_database_get_chemicals.server_timeout = std::chrono::milliseconds(2000);
  params_database_get_chemicals.wait_for_server_timeout = std::chrono::milliseconds(1000);
  params_database_get_chemicals.default_port_value = "get_chemical";
  factory.registerNodeType<GetChemicalNode>("GetChemicalNode",params_database_get_chemicals);

  RosNodeParams params_database_get_vessel;
  // params_database_get_vessel.nh = shared_from_this();
  params_database_get_vessel.nh = this->node_;
  params_database_get_vessel.server_timeout = std::chrono::milliseconds(2000);
  params_database_get_vessel.wait_for_server_timeout = std::chrono::milliseconds(1000);
  params_database_get_vessel.default_port_value = "get_vessel";
  factory.registerNodeType<GetVesselNode>("GetVesselNode",params_database_get_vessel);

  RosNodeParams params_database_place_vessel;
  // params_database_place_vessel.nh = shared_from_this();
  params_database_place_vessel.nh = this->node_;
  params_database_place_vessel.server_timeout = std::chrono::milliseconds(2000);
  params_database_place_vessel.wait_for_server_timeout = std::chrono::milliseconds(1000);
  params_database_place_vessel.default_port_value = "place_vessel";
  factory.registerNodeType<PlaceVesselNode>("PlaceVesselNode",params_database_place_vessel);

  RosNodeParams params_database_place_chemical;
  // params_database_place_vessel.nh = shared_from_this();
  params_database_place_chemical.nh = this->node_;
  params_database_place_chemical.server_timeout = std::chrono::milliseconds(2000);
  params_database_place_chemical.wait_for_server_timeout = std::chrono::milliseconds(1000);
  params_database_place_chemical.default_port_value = "place_chemical";
  factory.registerNodeType<PlaceChemicalNode>("PlaceChemicalNode",params_database_place_chemical);

        RosNodeParams params_database_remove_chemical_placement;
        // params_database_place_vessel.nh = shared_from_this();
        params_database_remove_chemical_placement.nh = this->node_;
        params_database_remove_chemical_placement.server_timeout = std::chrono::milliseconds(2000);
        params_database_remove_chemical_placement.wait_for_server_timeout = std::chrono::milliseconds(1000);
        params_database_remove_chemical_placement.default_port_value = "remove_chemical_placement";
        factory.registerNodeType<RemoveChemicalPlacementNode>("RemoveChemicalPlacementNode",params_database_remove_chemical_placement);

        RosNodeParams params_database_remove_vessel_placement;
        // params_database_place_vessel.nh = shared_from_this();
        params_database_remove_vessel_placement.nh = this->node_;
        params_database_remove_vessel_placement.server_timeout = std::chrono::milliseconds(2000);
        params_database_remove_vessel_placement.wait_for_server_timeout = std::chrono::milliseconds(1000);
        params_database_remove_vessel_placement.default_port_value = "remove_vessel_placement";
        factory.registerNodeType<RemoveVesselPlacementNode>("RemoveVesselPlacementNode",params_database_remove_vessel_placement);







        
        RosNodeParams params_gripper;
        // params_gripper.nh = shared_from_this();
        params_gripper.nh = this->node_;
        params_gripper.server_timeout = std::chrono::milliseconds(2000);
        params_gripper.wait_for_server_timeout = std::chrono::milliseconds(2000);
        params_gripper.default_port_value = "gripper_service";
        factory.registerNodeType<GripperAction>("GripperAction",params_gripper);
        //////////////////////////////////////////////////////////////////////////////
        RosNodeParams params_gripper_franka_grasp;
        // params_gripper_franka_grasp.nh = shared_from_this();
        params_gripper_franka_grasp.nh = this->node_;
        params_gripper_franka_grasp.server_timeout = std::chrono::milliseconds(4000);
        params_gripper_franka_grasp.wait_for_server_timeout = std::chrono::milliseconds(8000);
        params_gripper_franka_grasp.default_port_value = "panda_gripper/grasp";
        factory.registerNodeType<FrankaGraspGripperAction>("FrankaGraspGripperAction",params_gripper_franka_grasp);
        //////////////////////////////////////////////////////////////////////////////
        RosNodeParams params_gripper_franka_homing;
        // params_gripper_franka_homing.nh = shared_from_this();
        params_gripper_franka_homing.nh = this->node_;
        params_gripper_franka_homing.server_timeout = std::chrono::milliseconds(2000);
        params_gripper_franka_homing.wait_for_server_timeout = std::chrono::milliseconds(4000);
        params_gripper_franka_homing.default_port_value = "panda_gripper/homing";
        factory.registerNodeType<FrankaHomeGripperAction>("FrankaHomeGripperAction",params_gripper_franka_homing);
        ////////////////////////////////////////////////////////////////////////////
        RosNodeParams params_gripper_franka_move;
        // params_gripper_franka_move.nh = shared_from_this();
        params_gripper_franka_move.nh = this->node_;
        params_gripper_franka_move.server_timeout = std::chrono::milliseconds(2000);
        params_gripper_franka_move.wait_for_server_timeout = std::chrono::milliseconds(1000);
        params_gripper_franka_move.default_port_value = "panda_gripper/move";
        factory.registerNodeType<FrankaMoveGripperAction>("FrankaMoveGripperAction",params_gripper_franka_move);
        ////////////////////////////////////////////////////////////////////////////////////////
        RosNodeParams params_aruco;
        // params_aruco.nh = shared_from_this();
        params_aruco.nh = this->node_;
        params_aruco.server_timeout = std::chrono::milliseconds(2000);
        params_aruco.wait_for_server_timeout = std::chrono::milliseconds(1000);
        params_aruco.default_port_value = "detect_marker_pose";
        factory.registerNodeType<FindArucoTagAction>("ArucoAction",params_aruco);
        ////////////////////////////////////////////////////////////////////////////////////////
        RosNodeParams params_arm_mode_pose;
        // params_arm_mode_pose.nh = shared_from_this();
        params_arm_mode_pose.nh = this->node_;
        params_arm_mode_pose.server_timeout = std::chrono::milliseconds(2000);
        params_arm_mode_pose.wait_for_server_timeout = std::chrono::milliseconds(1000);
        params_arm_mode_pose.default_port_value = "arm_move_pose_service";
        factory.registerNodeType<ArmMovePoseAction>("ArmMovePoseAction",params_arm_mode_pose);

        RosNodeParams params_arm_mode_joints;
        // params_arm_mode_joints.nh = shared_from_this();
        params_arm_mode_joints.nh = this->node_;
        params_arm_mode_joints.server_timeout = std::chrono::milliseconds(2000);
        params_arm_mode_joints.wait_for_server_timeout = std::chrono::milliseconds(1000);
        params_arm_mode_joints.default_port_value = "arm_move_joints_service";
        factory.registerNodeType<ArmMoveJointsAction>("ArmMoveJointsAction",params_arm_mode_joints);

        RosNodeParams params_arm_move_joints_relative;
        params_arm_move_joints_relative.nh = this->node_;
        params_arm_move_joints_relative.server_timeout = std::chrono::milliseconds(2000);
        params_arm_move_joints_relative.wait_for_server_timeout = std::chrono::milliseconds(1000);
        params_arm_move_joints_relative.default_port_value = "arm_move_joints_relative_service";
        factory.registerNodeType<ArmMoveJointsRelativeAction>("ArmMoveJointsRelativeAction",params_arm_move_joints_relative);

        RosNodeParams params_gripper_joint;
        // params_gripper_joint.nh = shared_from_this();
        params_gripper_joint.nh = this->node_;

        params_gripper_joint.default_port_value = "gripper_joint_service";
        factory.registerNodeType<GripperJointAction>("GripperJointAction",params_gripper_joint);
        
        factory.registerNodeType<ArmArrayToPoseAction>("ArmArrayToPoseAction");
        factory.registerNodeType<ArmPoseMsgOffsetCalculation>("ArmPoseMsgOffsetCalculation");

        RosNodeParams params_arm_move_pose_msg;
        // params_arm_move_pose_msg.nh = shared_from_this();
        params_arm_move_pose_msg.nh = this->node_;
        params_arm_move_pose_msg.default_port_value = "arm_move_pose_msg_service";
        params_arm_move_pose_msg.server_timeout = std::chrono::milliseconds(2000);
        params_arm_move_pose_msg.wait_for_server_timeout = std::chrono::milliseconds(1000);
        factory.registerNodeType<ArmMovePoseMsgAction>("ArmMovePoseMsgAction",params_arm_move_pose_msg);

        RosNodeParams params_arm_move_pose_msg_tcp;
        // params_arm_move_pose_msg_tcp.nh = shared_from_this();
        params_arm_move_pose_msg_tcp.nh = this->node_;
        params_arm_move_pose_msg_tcp.default_port_value = "arm_move_pose_msg_tcp_service";
        params_arm_move_pose_msg_tcp.server_timeout = std::chrono::milliseconds(2000);
        params_arm_move_pose_msg_tcp.wait_for_server_timeout = std::chrono::milliseconds(1000);
        factory.registerNodeType<ArmMovePoseMsgTcpAction>("ArmMovePoseMsgTcpAction",params_arm_move_pose_msg_tcp);

        RosNodeParams params_arm_move_pliz_ptp_pose_msg;
        // params_arm_move_pliz_ptp_pose_msg.nh = shared_from_this();
        params_arm_move_pliz_ptp_pose_msg.nh = this->node_;
        params_arm_move_pliz_ptp_pose_msg.default_port_value = "arm_move_pliz_ptp_pose_msg_service";
        params_arm_move_pliz_ptp_pose_msg.server_timeout = std::chrono::milliseconds(2000);
        params_arm_move_pliz_ptp_pose_msg.wait_for_server_timeout = std::chrono::milliseconds(1000);
        factory.registerNodeType<ArmMovePlizPtpPoseMsgAction>("ArmMovePlizPtpPoseMsgAction",params_arm_move_pliz_ptp_pose_msg);


        RosNodeParams params_arm_move_pliz_lin_pose_msg;
        // params_arm_move_pliz_lin_pose_msg.nh = shared_from_this();
        params_arm_move_pliz_lin_pose_msg.nh = this->node_;
        params_arm_move_pliz_lin_pose_msg.default_port_value = "arm_move_pliz_lin_pose_msg_service";
        params_arm_move_pliz_lin_pose_msg.server_timeout = std::chrono::milliseconds(2000);
        params_arm_move_pliz_lin_pose_msg.wait_for_server_timeout = std::chrono::milliseconds(1000);
        factory.registerNodeType<ArmMovePlizLinPoseMsgAction>("ArmMovePlizLinPoseMsgAction",params_arm_move_pliz_lin_pose_msg);


        RosNodeParams params;
        // params.nh = shared_from_this();
        params.nh = this->node_;
        params.server_timeout = std::chrono::milliseconds(2000);
        params.wait_for_server_timeout = std::chrono::milliseconds(1000);
        params.default_port_value = "sleep_service";

        factory.registerNodeType<ArmPoseOffsetCalculation>("ArmPoseOffsetCalculation");

        RosNodeParams params_home_arm;
        // params_home_arm.nh = shared_from_this();
        params_home_arm.nh = this->node_;
        params_home_arm.server_timeout = std::chrono::milliseconds(2000);
        params_home_arm.wait_for_server_timeout = std::chrono::milliseconds(1000);
        params_home_arm.default_port_value = "home_arm";

        factory.registerNodeType<HomeArmAction>("HomeArmAction",params_home_arm);

        RosNodeParams params_clear_octomap;
        params_clear_octomap.nh = this->node_;
        params_clear_octomap.server_timeout = std::chrono::milliseconds(2000);
        params_clear_octomap.wait_for_server_timeout = std::chrono::milliseconds(1000);
        params_clear_octomap.default_port_value = "clear_octomap";
        factory.registerNodeType<ClearOctomapNode>("ClearOctomapNode",params_clear_octomap);

        RosNodeParams params_add_object;
        params_add_object.nh = this->node_;
        params_add_object.server_timeout = std::chrono::milliseconds(2400);
        params_add_object.wait_for_server_timeout = std::chrono::milliseconds(2000);
        params_add_object.default_port_value = "add_object_service";
        factory.registerNodeType<AddObjectNode>("AddObjectNode",params_add_object);

        RosNodeParams params_remove_object;
        params_remove_object.nh = this->node_;
        params_remove_object.server_timeout = std::chrono::milliseconds(4000);
        params_remove_object.wait_for_server_timeout = std::chrono::milliseconds(2000);
        params_remove_object.default_port_value = "remove_object_service";
        factory.registerNodeType<RemoveObjectNode>("RemoveObjectNode",params_remove_object);

        RosNodeParams params_attach_object;
        params_attach_object.nh = this->node_;
        params_attach_object.server_timeout = std::chrono::milliseconds(4000);
        params_attach_object.wait_for_server_timeout = std::chrono::milliseconds(2000);
        params_attach_object.default_port_value = "attach_object_service";
        factory.registerNodeType<AttachObjectNode>("AttachObjectNode",params_attach_object);

        RosNodeParams params_detach_object;
        params_detach_object.nh = this->node_;
        params_detach_object.server_timeout = std::chrono::milliseconds(4000);
        params_detach_object.wait_for_server_timeout = std::chrono::milliseconds(2000);
        params_detach_object.default_port_value = "detach_object_service";
        factory.registerNodeType<DetachObjectNode>("DetachObjectNode",params_detach_object);

        RosNodeParams params_get_pre_pour_pose;
        params_get_pre_pour_pose.nh = this->node_;
        params_get_pre_pour_pose.server_timeout = std::chrono::milliseconds(4000);
        params_get_pre_pour_pose.wait_for_server_timeout = std::chrono::milliseconds(2000);
        params_get_pre_pour_pose.default_port_value = "get_pre_pour_pose_service";
        factory.registerNodeType<GetPrePourPoseNode>("GetPrePourPoseNode",params_get_pre_pour_pose);


        RosNodeParams params_arm_move_trajectory_pour_msg;
        params_arm_move_trajectory_pour_msg.nh = this->node_;
        params_arm_move_trajectory_pour_msg.default_port_value = "arm_move_trajectory_pour_service";
        params_arm_move_trajectory_pour_msg.server_timeout = std::chrono::milliseconds(3000);
        params_arm_move_trajectory_pour_msg.wait_for_server_timeout = std::chrono::milliseconds(2000);
        factory.registerNodeType<ArmMoveTrajectoryPourAction>("ArmMoveTrajectoryPourAction",params_arm_move_trajectory_pour_msg);
        #ifdef USE_SLEEP_PLUGIN
        RegisterRosNode(factory, "../lib/libsleep_action_plugin.so", params);
        #else
        factory.registerNodeType<SleepAction>("SleepAction", params);
        #endif


  factory.registerBehaviorTreeFromFile(ament_index_cpp::get_package_share_directory("behavior_tree_ros2_rob8") + "/bt_xml/panda_test.xml");
  factory.registerBehaviorTreeFromFile(ament_index_cpp::get_package_share_directory("behavior_tree_ros2_rob8") + "/bt_xml/stirring.xml");
  factory.registerBehaviorTreeFromFile(ament_index_cpp::get_package_share_directory("behavior_tree_ros2_rob8") + "/bt_xml/mobile_base_trees.xml");
  factory.registerBehaviorTreeFromFile(ament_index_cpp::get_package_share_directory("behavior_tree_ros2_rob8") + "/bt_xml/manipulator_trees.xml");
  factory.registerBehaviorTreeFromFile(ament_index_cpp::get_package_share_directory("behavior_tree_ros2_rob8") + "/bt_xml/liquid_handling.xml");
  factory.registerBehaviorTreeFromFile(ament_index_cpp::get_package_share_directory("behavior_tree_ros2_rob8") + "/bt_xml/database_trees.xml");

    // BehaviorServer::test();
    }

  private:
    void Xdl_service(const std::shared_ptr<behavior_tree_ros2_actions::srv::Xdl::Request> request, const std::shared_ptr<behavior_tree_ros2_actions::srv::Xdl::Response> response)
    {
        factory.registerBehaviorTreeFromText(request->xdl);
        
        std::cout << "register main tree success\n";
        tree_ = factory.createTree("Main");
        std::cout << "create main tree success\n";

          //std::string xml_models = BT::writeTreeNodesModelXML(factory);
            //std::cout << "----------- XML file  ----------\n"
            //<< xml_models
            //<< "--------------------------------\n";



        
        // std::cout << BT::writeTreeToXML(tree);
        // std::cout << "----------- XML file  ----------\n"
        //     << BT::WriteTreeToXML(tree_, false, false)
        //     << "--------------------------------\n";
        

        response->result = true;
        publisher_ptr_ = std::make_unique<BT::Groot2Publisher>(tree_, 5555);
        const auto timer_period = 100ms;
            timer_ = this->create_wall_timer(
                timer_period,
                std::bind(&BehaviorServer::update_behavior_tree, this));

        // tree.tickWhileRunning();
        


    }
    void update_behavior_tree() {
            // Tick the behavior tree.
            BT::NodeStatus tree_status = tree_.tickOnce();
            if (tree_status == BT::NodeStatus::RUNNING) {
                return;
            }
            // Cancel the timer if we hit a terminal state.
            if (tree_status == BT::NodeStatus::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Finished with status SUCCESS");
                timer_->cancel();
                rclcpp::shutdown(); // remove this when using the service
            } else if (tree_status == BT::NodeStatus::FAILURE) {
                RCLCPP_INFO(this->get_logger(), "Finished with status FAILURE");
                timer_->cancel();
                rclcpp::shutdown(); // remove this when using the service
            }
        }

    void test(){
        tree_ = factory.createTree("node_test_tree");

          std::string xml_models = BT::writeTreeNodesModelXML(factory);
            std::cout << "----------- XML file  ----------\n"
            << xml_models
            << "--------------------------------\n";




        // std::cout << BT::writeTreeToXML(tree);
        std::cout << "----------- XML file  ----------\n"
            << BT::WriteTreeToXML(tree_, false, false)
            << "--------------------------------\n";
        
        publisher_ptr_ = std::make_unique<BT::Groot2Publisher>(tree_, 5555);
        const auto timer_period = 100ms;
            timer_ = this->create_wall_timer(
                timer_period,
                std::bind(&BehaviorServer::update_behavior_tree, this));
    }
    rclcpp::Service<behavior_tree_ros2_actions::srv::Xdl>::SharedPtr service_;
    rclcpp::Node::SharedPtr node_;
    BT::Tree tree_;
    std::unique_ptr<BT::Groot2Publisher> publisher_ptr_;
    BehaviorTreeFactory factory;
    rclcpp::Executor::SharedPtr executor_;
    std::thread executor_thread_;
    rclcpp::TimerBase::SharedPtr timer_;
};




//-----------------------------------------------------
const std::string default_bt_xml_foler = 
    ament_index_cpp::get_package_share_directory("behavior_tree_ros2_rob8") + "/bt_xml";

const std::string default_bt_xml_file = 
    ament_index_cpp::get_package_share_directory("behavior_tree_ros2_rob8") + "/bt_xml/panda_test.xml";



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto behavior_node = std::make_shared<BehaviorServer>(node_options);
    rclcpp::spin(behavior_node);

    rclcpp::shutdown();

  return 0;
}
