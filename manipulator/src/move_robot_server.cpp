#include "manipulator/move_robot_server.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <memory>


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// #include "ar4_msgs/action/home.hpp"

#include "behaviortree_ros2/bt_action_node.hpp"

static const double PLANNING_TIME_S = 5.0;
static const double MAX_VELOCITY_SCALE = 0.8;
static const double MAX_ACCELERATION_SCALE = 0.8;
static const unsigned int PLANNING_ATTEMPTS = 5;
static const double GOAL_TOLERANCE = 1e-3;
static const std::string PLANNING_GROUP = "panda_1";

static const std::string PLANNING_GRIPPER_GROUP = "panda_1_gripper";
static const std::vector<std::string> gripper_joint_names = {"panda1_finger_joint1", "panda1_finger_joint2"}; //{"gripper_joint1", "gripper_joint2"};
static const std::vector<std::string> arm_joint_names = {"panda1_joint1", "panda1_joint2", "panda1_joint3", "panda1_joint4", "panda1_joint5", "panda1_joint6", "panda1_joint7"};
// static const std::string PLANNING_GROUP = "arm_group";

// static const std::string PLANNING_GRIPPER_GROUP = "gripper";

// using std::placeholders::_1;
using namespace std::placeholders;

MoveRobotServer::MoveRobotServer(const rclcpp::NodeOptions &options)
        : Node("example_services_node", options), node_(std::make_shared<rclcpp::Node>("example_group_node")),
          executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) {
    node_namespace_ = ((std::string) this->get_namespace()).erase(0, 1);
    
    // service_example_server_ = this->create_service<MoveRobotServer::srv::ExampleService>(
    //         "/robot_service",
    //         std::bind(&MoveRobotServer::service_example_callback, this, std::placeholders::_1, std::placeholders::_2)
    // );

    // auto mgi_options = moveit::planning_interface::MoveGroupInterface::Options(
    //         node_namespace_ + "_ur_manipulator",
    //         "/" + node_namespace_,
    //         "robot_description");
    this->action_server_home_arm_ = rclcpp_action::create_server<Home>(
      this,
      "home_arm",
      std::bind(&MoveRobotServer::home_arm_handle_goal, this, _1, _2),
      std::bind(&MoveRobotServer::home_arm_handle_cancel, this, _1),
      std::bind(&MoveRobotServer::home_arm_handle_accepted, this, _1));
    
    this->action_server_gripper_ = rclcpp_action::create_server<Gripper>(
      this,
      "gripper_service",
      std::bind(&MoveRobotServer::gripper_handle_goal, this, _1, _2),
      std::bind(&MoveRobotServer::gripper_handle_cancel, this, _1),
      std::bind(&MoveRobotServer::gripper_handle_accepted, this, _1));

    this->action_server_arm_move_pose_ = rclcpp_action::create_server<ArmMovePose>(
      this,
      "arm_move_pose_service",
      std::bind(&MoveRobotServer::arm_move_pose_handle_goal, this, _1, _2),
      std::bind(&MoveRobotServer::arm_move_pose_handle_cancel, this, _1),
      std::bind(&MoveRobotServer::arm_move_pose_handle_accepted, this, _1));
    
    this->action_server_arm_move_pose_msg_ = rclcpp_action::create_server<ArmMovePoseMsg>(
      this,
      "arm_move_pose_msg_service",
      std::bind(&MoveRobotServer::arm_move_pose_msg_handle_goal, this, _1, _2),
      std::bind(&MoveRobotServer::arm_move_pose_msg_handle_cancel, this, _1),
      std::bind(&MoveRobotServer::arm_move_pose_msg_handle_accepted, this, _1));

    this->action_server_arm_move_joints_ = rclcpp_action::create_server<ArmMoveJoints>(
      this,
      "arm_move_joints_service",
      std::bind(&MoveRobotServer::arm_move_joints_handle_goal, this, _1, _2),
      std::bind(&MoveRobotServer::arm_move_joints_handle_cancel, this, _1),
      std::bind(&MoveRobotServer::arm_move_joints_handle_accepted, this, _1));
    
    this->action_server_sleep_ = rclcpp_action::create_server<Sleep>(
      this,
      "sleep_service",
      std::bind(&MoveRobotServer::sleep_handle_goal, this, _1, _2),
      std::bind(&MoveRobotServer::sleep_handle_cancel, this, _1),
      std::bind(&MoveRobotServer::sleep_handle_accepted, this, _1));
    this->action_server_gripper_joint_ = rclcpp_action::create_server<GripperJoint>(
      this,
      "gripper_joint_service",
      std::bind(&MoveRobotServer::gripper_joint_handle_goal, this, _1, _2),
      std::bind(&MoveRobotServer::gripper_joint_handle_cancel, this, _1),
      std::bind(&MoveRobotServer::gripper_joint_handle_accepted, this, _1));
    
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "topic", 10, std::bind(&MoveRobotServer::Move, this, _1));
    
    subscription_gripper_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "gripper", 10, std::bind(&MoveRobotServer::MoveGripper, this, _1));

    move_gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, PLANNING_GRIPPER_GROUP);
    
    move_gripper_group_->setPlanningPipelineId("ompl");

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, PLANNING_GROUP);
    //move_group_->setPoseReferenceFrame("base_link_inertia");
    move_group_->setPlanningTime(PLANNING_TIME_S);
    move_group_->setNumPlanningAttempts(PLANNING_ATTEMPTS);
    move_group_->setGoalTolerance(GOAL_TOLERANCE);
    move_group_->setMaxVelocityScalingFactor(MAX_VELOCITY_SCALE);
    move_group_->setMaxAccelerationScalingFactor(MAX_ACCELERATION_SCALE);
    move_group_->setPlanningPipelineId("ompl");
    // move_group_->setPlannerId("PTP");
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { this->executor_->spin(); });
}

bool MoveRobotServer::ArmMoveJ(const std_msgs::msg::Float64MultiArray & msg){
  std::vector<double> joints;
  for(int i = 0; i<msg.data.size();i++)
  {
    joints.push_back(msg.data[i]);
  }
  // for(int i = 0; i < v.size(); i++)
  //   {
  //     pos.push_back(stod(v[i]));
  //   }
  move_group_->setJointValueTarget(arm_joint_names,joints);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = static_cast<bool>(move_group_->plan(my_plan));
              RCLCPP_INFO(this->get_logger(), " (MoveJ movement) %s", success ? "" : "FAILED");
              if(success == true){
                  //move_group.move();
                  move_gripper_group_->execute(my_plan);
                  return true;
              }
              else{
                return false;
              }
}

bool MoveRobotServer::MoveGripper(const std_msgs::msg::Float64MultiArray & msg)
{
  // std::vector<double> joints = {-0.628318531, 0.628318531};
  // std::vector<std::string> joint_names = {"gripper_joint1", "gripper_joint2"};
  std::vector<double> joints = {msg.data[0], msg.data[1]};
  
  // joint_names = {"gripper_joint1", "gripper_joint2"};
  move_gripper_group_->setJointValueTarget(gripper_joint_names,joints);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = static_cast<bool>(move_gripper_group_->plan(my_plan));
              RCLCPP_INFO(this->get_logger(), " (movement) %s", success ? "" : "FAILED");
              if(success == true){
                  //move_group.move();
                  move_gripper_group_->execute(my_plan);
                  return true;
              }
              else{
                return false;
              }


}

bool MoveRobotServer::Move(const geometry_msgs::msg::PoseStamped & msg)
{
    RCLCPP_INFO(this->get_logger(), "move function called");
    move_group_->setPoseReferenceFrame("panda1_link0");
    geometry_msgs::msg::Pose pose_goal; // = move_group_->getCurrentPose().pose;
    // pose_goal.position.x = 0.1;;
    // pose_goal.position.y = -0.4;
    // pose_goal.position.z = 0.4;
    // // tf2::Quaternion q_orig(pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z,
    // //                        pose_goal.orientation.w);
    // // tf2::Quaternion q_rot(request->transform.rotation.x, request->transform.rotation.y, request->transform.rotation.z,
    // //                        request->transform.rotation.w);
    // tf2::Quaternion q_new;
    // q_new.setRPY(0.0, 1.529, -1.529);
    
    // //q_new = q_rot * q_orig;
    // q_new.normalize();
    // pose_goal.orientation.x = q_new.x();
    // pose_goal.orientation.y = q_new.y();
    // pose_goal.orientation.z = q_new.z();
    // pose_goal.orientation.w = q_new.w();
    pose_goal.position.x = msg.pose.position.x;
    pose_goal.position.y = msg.pose.position.y;
    pose_goal.position.z = msg.pose.position.z;
    // tf2::Quaternion q_orig(pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z,
    //                        pose_goal.orientation.w);
    // tf2::Quaternion q_rot(request->transform.rotation.x, request->transform.rotation.y, request->transform.rotation.z,
    //                        request->transform.rotation.w);
    tf2::Quaternion q_new;
    q_new.setRPY(0.0, 1.529, -1.529);
    
    //q_new = q_rot * q_orig;
    q_new.normalize();
    pose_goal.orientation.x = msg.pose.orientation.x;
    pose_goal.orientation.y = msg.pose.orientation.y;
    pose_goal.orientation.z = msg.pose.orientation.z;
    pose_goal.orientation.w = msg.pose.orientation.w;
    // move_group_->setStartStateToCurrentState();
    // move_group_->setPoseTarget(pose_goal);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    RCLCPP_INFO(this->get_logger(), "Planned position x: %f, y: %f, z: %f", pose_goal.position.x, pose_goal.position.y,
              pose_goal.position.z);
    // bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // RCLCPP_INFO(this->get_logger(), " (movement) %s", success ? "" : "FAILED");
    // move_group_->move();
    move_group_->setStartStateToCurrentState();
    move_group_->setPoseTarget(pose_goal);
    bool success = static_cast<bool>(move_group_->plan(my_plan));
              RCLCPP_INFO(this->get_logger(), " (movement) %s", success ? "" : "FAILED");
              if(success == true){
                  //move_group.move();
                  // move_group_->setStartStateToCurrentState();
                  if(move_group_->execute(my_plan).val == 1){
                    return true;
                  }
                  else{
                    move_group_->setStartStateToCurrentState();
                    move_group_->setPoseTarget(pose_goal);
                    bool success = static_cast<bool>(move_group_->plan(my_plan));
                      RCLCPP_INFO(this->get_logger(), " (movement) %s", success ? "" : "FAILED");
                    if(success == true){
                      // move_group.move();
                      move_group_->setStartStateToCurrentState();
                      if(move_group_->execute(my_plan).val == 1){
                        return true;
                      }
                      else{
                        return false;
                      }
                      
                    }

                  }
              }
              else{
                  return false;
                }
                
}

rclcpp_action::GoalResponse MoveRobotServer::home_arm_handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Home::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal requesdddt with sleep time %d", goal->msec_timeout);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse MoveRobotServer::home_arm_handle_cancel(
    const std::shared_ptr<GoalHandleHome> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void MoveRobotServer::home_arm_handle_accepted(const std::shared_ptr<GoalHandleHome> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    // std::thread{std::bind(&SleepActionServer::execute, this, _1), goal_handle}.detach();
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Home::Feedback>();
    auto result = std::make_shared<Home::Result>();
    result->done = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");

  }


rclcpp_action::GoalResponse MoveRobotServer::gripper_joint_handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const GripperJoint::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal requsdadest with sleep time %d",goal->joint_1);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse MoveRobotServer::gripper_joint_handle_cancel(
    const std::shared_ptr<GoalHandleGripperJoint> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void MoveRobotServer::gripper_joint_handle_accepted(const std::shared_ptr<GoalHandleGripperJoint> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&MoveRobotServer::gripper_joint_execute, this, _1), goal_handle}.detach();
    // auto result = std::make_shared<Gripper::Result>();
    //   std_msgs::msg::Float64MultiArray joints;
    //   const auto goal = goal_handle->get_goal();
    //   bool open = goal->open;
    //   if(open == true)
    //   {
    //     joints.data = {-0.628318531, 0.628318531};
    //     RCLCPP_INFO(this->get_logger(), "opening %d",goal->open);
    //     if(MoveRobotServer::MoveGripper(joints)){
          
    //       result->done = true;
    //       goal_handle->succeed(result);
    //       RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    //     }
    //     // else{
    //     //   result->done = false;
    //     //   goal_handle->canceled(result);
    //     //   RCLCPP_INFO(this->get_logger(), "Goal canceled");
    //     // }

    //   }
    //   if(open == false)
    //   {
        
    //     joints.data = {0.0, 0.0};
    //     RCLCPP_INFO(this->get_logger(), "closing %d",goal->open);
    //     if(MoveRobotServer::MoveGripper(joints)){
    //       result->done = true;
    //       goal_handle->succeed(result);
    //       RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    //     }
    //     // else{
    //     //   result->done = false;
    //     //   goal_handle->canceled(result);
    //     //   RCLCPP_INFO(this->get_logger(), "Goal canceled");
    //     // }

    //   }
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    // std::thread{std::bind(&SleepActionServer::execute, this, _1), goal_handle}.detach();
  }

  void MoveRobotServer::gripper_joint_execute(const std::shared_ptr<GoalHandleGripperJoint> goal_handle){
      auto result = std::make_shared<GripperJoint::Result>();
      std_msgs::msg::Float64MultiArray joints;
      const auto goal = goal_handle->get_goal();
      // bool open = goal->open;
    //   const auto goal = goal_handle->get_goal();
    //   auto pose = goal->pose;
    //   std::vector<std::string> v;
 
    //   std::stringstream ss(pose);
    //   std::vector<double> pos;
    // while (ss.good()) {
    //     std::string substr;
    //     getline(ss, substr, ',');
    //     v.push_back(substr);
    // }
    // for(int i = 0; i < v.size(); i++)
    // {
    //   pos.push_back(stod(v[i]));
    // }
    
        joints.data = {goal->joint_1, goal->joint_2};
        RCLCPP_INFO(this->get_logger(), "opening %d",goal->joint_1);
        if(MoveRobotServer::MoveGripper(joints)){
          
          result->done = true;
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
        else{
          result->done = false;
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
        }
  }




rclcpp_action::GoalResponse MoveRobotServer::gripper_handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Gripper::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal requsdadest with sleep time %d",goal->open);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse MoveRobotServer::gripper_handle_cancel(
    const std::shared_ptr<GoalHandleGripper> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void MoveRobotServer::gripper_handle_accepted(const std::shared_ptr<GoalHandleGripper> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&MoveRobotServer::gripper_execute, this, _1), goal_handle}.detach();
    // auto result = std::make_shared<Gripper::Result>();
    //   std_msgs::msg::Float64MultiArray joints;
    //   const auto goal = goal_handle->get_goal();
    //   bool open = goal->open;
    //   if(open == true)
    //   {
    //     joints.data = {-0.628318531, 0.628318531};
    //     RCLCPP_INFO(this->get_logger(), "opening %d",goal->open);
    //     if(MoveRobotServer::MoveGripper(joints)){
          
    //       result->done = true;
    //       goal_handle->succeed(result);
    //       RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    //     }
    //     // else{
    //     //   result->done = false;
    //     //   goal_handle->canceled(result);
    //     //   RCLCPP_INFO(this->get_logger(), "Goal canceled");
    //     // }

    //   }
    //   if(open == false)
    //   {
        
    //     joints.data = {0.0, 0.0};
    //     RCLCPP_INFO(this->get_logger(), "closing %d",goal->open);
    //     if(MoveRobotServer::MoveGripper(joints)){
    //       result->done = true;
    //       goal_handle->succeed(result);
    //       RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    //     }
    //     // else{
    //     //   result->done = false;
    //     //   goal_handle->canceled(result);
    //     //   RCLCPP_INFO(this->get_logger(), "Goal canceled");
    //     // }

    //   }
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    // std::thread{std::bind(&SleepActionServer::execute, this, _1), goal_handle}.detach();
  }

  void MoveRobotServer::gripper_execute(const std::shared_ptr<GoalHandleGripper> goal_handle){
      auto result = std::make_shared<Gripper::Result>();
      std_msgs::msg::Float64MultiArray joints;
      const auto goal = goal_handle->get_goal();
      bool open = goal->open;
      if(open == true)
      {
        joints.data = {-0.628318531, 0.628318531};
        RCLCPP_INFO(this->get_logger(), "opening %d",goal->open);
        if(MoveRobotServer::MoveGripper(joints)){
          
          result->done = true;
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
        else{
          result->done = false;
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
        }

      }
      if(open == false)
      {
        
        joints.data = {0.0, 0.0};
        RCLCPP_INFO(this->get_logger(), "closing %d",goal->open);
        if(MoveRobotServer::MoveGripper(joints)){
          result->done = true;
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
        else{
          result->done = false;
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
        }

      }
  }


////////// ARM ACTION SERVICES //////////////7

rclcpp_action::GoalResponse MoveRobotServer::arm_move_pose_msg_handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ArmMovePoseMsg::Goal> goal)
  {
    // RCLCPP_INFO(this->get_logger(), "Received goal requsdadest with sleep time %d",goal->pose);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse MoveRobotServer::arm_move_pose_msg_handle_cancel(
    const std::shared_ptr<GoalHandleArmMovePoseMsg> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void MoveRobotServer::arm_move_pose_msg_handle_accepted(const std::shared_ptr<GoalHandleArmMovePoseMsg> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&MoveRobotServer::arm_move_pose_msg_execute, this, _1), goal_handle}.detach();
  
  }

  void MoveRobotServer::arm_move_pose_msg_execute(const std::shared_ptr<GoalHandleArmMovePoseMsg> goal_handle){
      auto result = std::make_shared<ArmMovePoseMsg::Result>();
      
      const auto goal = goal_handle->get_goal();
      auto pose = goal->pose;
      
      if(MoveRobotServer::Move(pose))
      {
          result->done = true;
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      }
      else{
          result->done = false;
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
      }
      // if(open == true)
      // {
      //   joints.data = {-0.628318531, 0.628318531};
      //   RCLCPP_INFO(this->get_logger(), "opening %d",goal->open);
      //   if(MoveRobotServer::MoveGripper(joints)){
          
      //     result->done = true;
      //     goal_handle->succeed(result);
      //     RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      //   }
      //   else{
      //     result->done = false;
      //     goal_handle->canceled(result);
      //     RCLCPP_INFO(this->get_logger(), "Goal canceled");
      //   }

      // }
      // if(open == false)
      // {
        
      //   joints.data = {0.0, 0.0};
      //   RCLCPP_INFO(this->get_logger(), "closing %d",goal->open);
      //   if(MoveRobotServer::MoveGripper(joints)){
      //     result->done = true;
      //     goal_handle->succeed(result);
      //     RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      //   }
      //   else{
      //     result->done = false;
      //     goal_handle->canceled(result);
      //     RCLCPP_INFO(this->get_logger(), "Goal canceled");
      //   }

      // }
  }


rclcpp_action::GoalResponse MoveRobotServer::arm_move_pose_handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ArmMovePose::Goal> goal)
  {
    // RCLCPP_INFO(this->get_logger(), "Received goal requsdadest with sleep time %d",goal->pose);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse MoveRobotServer::arm_move_pose_handle_cancel(
    const std::shared_ptr<GoalHandleArmMovePose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void MoveRobotServer::arm_move_pose_handle_accepted(const std::shared_ptr<GoalHandleArmMovePose> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&MoveRobotServer::arm_move_pose_execute, this, _1), goal_handle}.detach();
  
  }

  void MoveRobotServer::arm_move_pose_execute(const std::shared_ptr<GoalHandleArmMovePose> goal_handle){
      auto result = std::make_shared<ArmMovePose::Result>();
      std_msgs::msg::Float64MultiArray joints;
      const auto goal = goal_handle->get_goal();
      auto pose = goal->pose;
    //   std::vector<std::string> v;
 
    //   std::stringstream ss(pose);
      std::vector<double> pos;
    // while (ss.good()) {
    //     std::string substr;
    //     getline(ss, substr, ',');
    //     v.push_back(substr);
    // }
    for(int i = 0; i < pose.size(); i++)
    {
      pos.push_back(pose[i]);
    }
      geometry_msgs::msg::PoseStamped pose_goal; // = move_group_->getCurrentPose().pose;
   
      pose_goal.pose.position.x = pos[0];
      pose_goal.pose.position.y = pos[1];
      pose_goal.pose.position.z = pos[2];

      tf2::Quaternion q_new;
      q_new.setRPY(pos[3], pos[4], pos[5]);
    

      q_new.normalize();
      
      pose_goal.pose.orientation.x = q_new.x();
      pose_goal.pose.orientation.y = q_new.y();
      pose_goal.pose.orientation.z = q_new.z();
      pose_goal.pose.orientation.w = q_new.w();
      
      if(MoveRobotServer::Move(pose_goal))
      {
          result->done = true;
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      }
      else{
          result->done = false;
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
      }
      // if(open == true)
      // {
      //   joints.data = {-0.628318531, 0.628318531};
      //   RCLCPP_INFO(this->get_logger(), "opening %d",goal->open);
      //   if(MoveRobotServer::MoveGripper(joints)){
          
      //     result->done = true;
      //     goal_handle->succeed(result);
      //     RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      //   }
      //   else{
      //     result->done = false;
      //     goal_handle->canceled(result);
      //     RCLCPP_INFO(this->get_logger(), "Goal canceled");
      //   }

      // }
      // if(open == false)
      // {
        
      //   joints.data = {0.0, 0.0};
      //   RCLCPP_INFO(this->get_logger(), "closing %d",goal->open);
      //   if(MoveRobotServer::MoveGripper(joints)){
      //     result->done = true;
      //     goal_handle->succeed(result);
      //     RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      //   }
      //   else{
      //     result->done = false;
      //     goal_handle->canceled(result);
      //     RCLCPP_INFO(this->get_logger(), "Goal canceled");
      //   }

      // }
  }


rclcpp_action::GoalResponse MoveRobotServer::arm_move_joints_handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ArmMoveJoints::Goal> goal)
  {
    // RCLCPP_INFO(this->get_logger(), "Received goal requsdadest with sleep time %d",goal->pose);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse MoveRobotServer::arm_move_joints_handle_cancel(
    const std::shared_ptr<GoalHandleArmMoveJoints> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void MoveRobotServer::arm_move_joints_handle_accepted(const std::shared_ptr<GoalHandleArmMoveJoints> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&MoveRobotServer::arm_move_joints_execute, this, _1), goal_handle}.detach();
  
  }

  void MoveRobotServer::arm_move_joints_execute(const std::shared_ptr<GoalHandleArmMoveJoints> goal_handle){
      auto result = std::make_shared<ArmMoveJoints::Result>();
      std_msgs::msg::Float64MultiArray joints_;
      const auto goal = goal_handle->get_goal();
      auto joint_pose = goal->joints;
      for(int i = 0; i<joint_pose.size();i++)
      {
        joints_.data.push_back(joint_pose[i]);
       }
      
      if(MoveRobotServer::ArmMoveJ(joints_))
      {
          result->done = true;
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      }
      else{
          result->done = false;
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
      }
  }

//////////  SLEEP ACTION //////////////

rclcpp_action::GoalResponse MoveRobotServer::sleep_handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Sleep::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with sleep time %d", goal->msec_timeout);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse MoveRobotServer::sleep_handle_cancel(
    const std::shared_ptr<GoalHandleSleep> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void MoveRobotServer::sleep_handle_accepted(const std::shared_ptr<GoalHandleSleep> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&MoveRobotServer::sleep_execute, this, _1), goal_handle}.detach();
  }

  void MoveRobotServer::sleep_execute(const std::shared_ptr<GoalHandleSleep> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(5);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Sleep::Feedback>();
    auto result = std::make_shared<Sleep::Result>();

    rclcpp::Time deadline = get_clock()->now() + rclcpp::Duration::from_seconds( double(goal->msec_timeout) / 1000 );
    int cycle = 0;

    while( get_clock()->now() < deadline )
    {
      if (goal_handle->is_canceling())
      {
        result->done = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      feedback->cycle = cycle++;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok())
    {
      result->done = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
// void MoveRobotServer::service_example_callback(const std::shared_ptr<ar4_moveit_config::srv::MoveRobotServer::Request> request,
//          std::shared_ptr<ar4_moveit_config::srv::MoveRobotServer::Response>      response) {
//     // move_group_->setStartStateToCurrentState();
//     // geometry_msgs::msg::Pose pose_goal = move_group_->getCurrentPose().pose;
//     // pose_goal.position.x += request->transform.translation.x;
//     // pose_goal.position.y += request->transform.translation.y;
//     // pose_goal.position.z += request->transform.translation.z;
//     // tf2::Quaternion q_orig(pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z,
//     //                        pose_goal.orientation.w);
//     // tf2::Quaternion q_rot(request->transform.rotation.x, request->transform.rotation.y, request->transform.rotation.z,
//     //                       request->transform.rotation.w);
//     // tf2::Quaternion q_new;
//     // q_new = q_rot * q_orig;
//     // q_new.normalize();
//     // pose_goal.orientation.x = q_new.x();
//     // pose_goal.orientation.y = q_new.y();
//     // pose_goal.orientation.z = q_new.z();
//     // pose_goal.orientation.w = q_new.w();
//     // move_group_->setPoseTarget(pose_goal);
//     // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//     // RCLCPP_INFO(this->get_logger(), "Planned position x: %f, y: %f, z: %f", pose_goal.position.x, pose_goal.position.y,
//     //             pose_goal.position.z);
//     // bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     // response->success = success;
//     std_msgs::msg::String i;
//     MoveRobotServer::Move(i);
//     reponse->sum = 1;
//     // move_group_->move();
// }

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = std::make_shared<MoveRobotServer>(node_options);
    rclcpp::spin(move_group_node);

    rclcpp::shutdown();
    return 0;
}
