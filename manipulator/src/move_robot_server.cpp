#include "manipulator/move_robot_server.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <memory>
#include <thread>
#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"

// #include "mtc_pour/pour_into.h"
// #include <moveit/task_constructor/moveit_compat.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/cartesian_interpolator.h>

#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <geometric_shapes/shape_extents.h>
#include <shape_msgs/msg/solid_primitive.h>
#include <tf2_eigen/tf2_eigen.hpp>


#include "rclcpp_action/rclcpp_action.hpp"
// #include "ar4_msgs/action/home.hpp"

#include "behaviortree_ros2/bt_action_node.hpp"
#include "std_srvs/srv/empty.hpp"

// #include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/moveit_cpp/moveit_cpp.h>



static const double PLANNING_TIME_S = 5.0;
static const double MAX_VELOCITY_SCALE = 0.8;
static const double MAX_ACCELERATION_SCALE = 0.8;
static const unsigned int PLANNING_ATTEMPTS = 10;
static const double GOAL_TOLERANCE = 1e-3;
// static const std::string PLANNING_GROUP = "panda_arm";
// static const std::string base_link = "panda_link0";
// static const std::string PLANNING_GRIPPER_GROUP = "hand";
// static const std::vector<std::string> gripper_joint_names = {"panda_finger_joint1", "panda_finger_joint2"}; //{"gripper_joint1", "gripper_joint2"};
// static const std::vector<std::string> arm_joint_names = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
static const std::string PLANNING_GROUP = "panda_arm";
static const std::string base_link = "panda_link0";
static const std::string PLANNING_GRIPPER_GROUP = "hand";
static const std::vector<std::string> gripper_joint_names = {"panda_finger_joint1", "panda_finger_joint2"}; //{"gripper_joint1", "gripper_joint2"};
static const std::vector<std::string> arm_joint_names = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};

// static const std::string PLANNING_GROUP = "arm_group";

// static const std::string PLANNING_GRIPPER_GROUP = "gripper";



// using std::placeholders::_1;
using namespace std::placeholders;

MoveRobotServer::MoveRobotServer(const rclcpp::NodeOptions &options)
        : Node("example_services_node", options), node_(std::make_shared<rclcpp::Node>("example_group_node")),
          executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
          {
    node_namespace_ = ((std::string) this->get_namespace()).erase(0, 1);

    // service_example_server_ = this->create_service<MoveRobotServer::srv::ExampleService>(
    //         "/robot_service",
    //         std::bind(&MoveRobotServer::service_example_callback, this, std::placeholders::_1, std::placeholders::_2)
    // );

    // auto mgi_options = moveit::planning_interface::MoveGroupInterface::Options(
    //         node_namespace_ + "_ur_manipulator",
    //         "/" + node_namespace_,
    //         "robot_description");
    this->add_object_srv_ = create_service<AddObject>(
        "add_object_service", std::bind(&MoveRobotServer::add_object_callback, this,
                                _1, _2));
    this->remove_object_srv_ = create_service<RemoveObject>(
        "remove_object_service", std::bind(&MoveRobotServer::remove_object_callback, this,
                                _1, _2));
    this->attach_object_srv_ = create_service<AttachObject>(
        "attach_object_service", std::bind(&MoveRobotServer::attach_object_callback, this,
                                _1, _2));
    this->detach_object_srv_ = create_service<DetachObject>(
        "detach_object_service", std::bind(&MoveRobotServer::detach_object_callback, this,
                                _1, _2));
    this->get_pre_pour_pose_srv_ = create_service<GetPrePourPose>(
        "get_pre_pour_pose_service", std::bind(&MoveRobotServer::get_pre_pour_pose_callback, this,
                                _1, _2));

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

    this->action_server_arm_move_pose_msg_tcp_ = rclcpp_action::create_server<ArmMovePoseMsgTcp>(
      this,
      "arm_move_pose_msg_tcp_service",
      std::bind(&MoveRobotServer::arm_move_pose_msg_tcp_handle_goal, this, _1, _2),
      std::bind(&MoveRobotServer::arm_move_pose_msg_tcp_handle_cancel, this, _1),
      std::bind(&MoveRobotServer::arm_move_pose_msg_tcp_handle_accepted, this, _1));

    this->action_server_arm_move_pliz_ptp_pose_msg_ = rclcpp_action::create_server<ArmMovePlizPtpPoseMsg>(
      this,
      "arm_move_pliz_ptp_pose_msg_service",
      std::bind(&MoveRobotServer::arm_move_pliz_ptp_pose_msg_handle_goal, this, _1, _2),
      std::bind(&MoveRobotServer::arm_move_pliz_ptp_pose_msg_handle_cancel, this, _1),
      std::bind(&MoveRobotServer::arm_move_pliz_ptp_pose_msg_handle_accepted, this, _1));

    this->action_server_arm_move_pliz_lin_pose_msg_ = rclcpp_action::create_server<ArmMovePlizLinPoseMsg>(
      this,
      "arm_move_pliz_lin_pose_msg_service",
      std::bind(&MoveRobotServer::arm_move_pliz_lin_pose_msg_handle_goal, this, _1, _2),
      std::bind(&MoveRobotServer::arm_move_pliz_lin_pose_msg_handle_cancel, this, _1),
      std::bind(&MoveRobotServer::arm_move_pliz_lin_pose_msg_handle_accepted, this, _1));

    this->action_server_arm_move_joints_ = rclcpp_action::create_server<ArmMoveJoints>(
      this,
      "arm_move_joints_service",
      std::bind(&MoveRobotServer::arm_move_joints_handle_goal, this, _1, _2),
      std::bind(&MoveRobotServer::arm_move_joints_handle_cancel, this, _1),
      std::bind(&MoveRobotServer::arm_move_joints_handle_accepted, this, _1));

    this->action_server_arm_move_joints_relative_ = rclcpp_action::create_server<ArmMoveJointsRelative>(
      this,
      "arm_move_joints_relative_service",
      std::bind(&MoveRobotServer::arm_move_joints_relative_handle_goal, this, _1, _2),
      std::bind(&MoveRobotServer::arm_move_joints_relative_handle_cancel, this, _1),
      std::bind(&MoveRobotServer::arm_move_joints_relative_handle_accepted, this, _1));

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

    this->action_server_gripper_joint_ = rclcpp_action::create_server<GripperJoint>(
      this,
      "gripper_joint_service",
      std::bind(&MoveRobotServer::gripper_joint_handle_goal, this, _1, _2),
      std::bind(&MoveRobotServer::gripper_joint_handle_cancel, this, _1),
      std::bind(&MoveRobotServer::gripper_joint_handle_accepted, this, _1));

    this->action_server_arm_move_trajectory_pour_ = rclcpp_action::create_server<ArmMoveTrajectoryPour>(
      this,
      "arm_move_trajectory_pour_service",
      std::bind(&MoveRobotServer::arm_move_trajectory_pour_handle_goal, this, _1, _2),
      std::bind(&MoveRobotServer::arm_move_trajectory_pour_handle_cancel, this, _1),
      std::bind(&MoveRobotServer::arm_move_trajectory_pour_handle_accepted, this, _1));

    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "topic", 10, std::bind(&MoveRobotServer::Move, this, _1));

    subscription_gripper_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "gripper", 10, std::bind(&MoveRobotServer::MoveGripper, this, _1));

    move_gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, PLANNING_GRIPPER_GROUP);

    move_gripper_group_->setPlanningPipelineId("ompl");

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, PLANNING_GROUP);
    move_group_->setPoseReferenceFrame(base_link);
    move_group_->setPlanningTime(PLANNING_TIME_S);
    move_group_->setNumPlanningAttempts(PLANNING_ATTEMPTS);
    move_group_->setGoalTolerance(GOAL_TOLERANCE);
    move_group_->setMaxVelocityScalingFactor(MAX_VELOCITY_SCALE);
    move_group_->setMaxAccelerationScalingFactor(MAX_ACCELERATION_SCALE);
    move_group_->setPlanningPipelineId("ompl");
    move_group_->setEndEffectorLink(tcp_frame); /// or move_group_->setEndEffector();

    move_group_->setSupportSurfaceName("base_plate_link");

    // move_group_->setPlannerId("PTP");
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { this->executor_->spin(); });

    auto robot_model_ = move_group_->getRobotModel();

    // visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(node_, base_link, "visual_markers", robot_model_);

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();

    text_pose.translation().z() = 1.0;
   //visual_tools->deleteAllMarkers();  // clear all old markers
   //visual_tools->publishText(text_pose, "Frank_________A", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
   //visual_tools->trigger();





    // ! https://github.com/ros-planning/moveit2_tutorials/blob/main/doc/examples/motion_planning_api/src/motion_planning_api_tutorial.cpp
}


void MoveRobotServer::add_object_callback(
      const std::shared_ptr<AddObject::Request> request,
      const std::shared_ptr<AddObject::Response> response){
        RCLCPP_INFO(this->get_logger(), "add_object_callback called");

        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        moveit_msgs::msg::CollisionObject object_to_attach;


        object_to_attach.id = request->object_id;


        shape_msgs::msg::SolidPrimitive primitive;

        if(request->shape == "box"){
          primitive.type = primitive.BOX;
          primitive.dimensions.resize(3);
          primitive.dimensions[primitive.BOX_X] = request->size_y;
          primitive.dimensions[primitive.BOX_Y] = request->size_y;
          primitive.dimensions[primitive.BOX_Z] = request->size_x;
        }
        else{

          primitive.type = primitive.CYLINDER;
          primitive.dimensions.resize(2);
          primitive.dimensions[primitive.CYLINDER_HEIGHT] = request->size_y;
          primitive.dimensions[primitive.CYLINDER_RADIUS] = request->size_x / 2.0;
        }

        // We define the frame/pose for this cylinder so that it appears in the gripper.
        object_to_attach.header.frame_id = move_group_->getPoseReferenceFrame();//move_group_->getEndEffectorLink();
        geometry_msgs::msg::Pose grab_pose;
        // grab_pose.orientation.w = 1.0;
        // grab_pose.position.z = 0.2;
        grab_pose = request->pose.pose;

        //! This enforces straigt orientation of objects
        grab_pose.orientation.x = 0.0;
        grab_pose.orientation.y = 0.0;
        grab_pose.orientation.z = 0.0;
        grab_pose.orientation.w = 1.0;


        grab_pose.position.z -= request->size_y/2 - 0.02;//0.011; //size of testtube/2-(height of gripper/2)-(offset)

        // First, we add the object to the world (without using a vector).
        object_to_attach.primitives.push_back(primitive);
        object_to_attach.primitive_poses.push_back(grab_pose);
        object_to_attach.operation = object_to_attach.ADD;

        // {  // Lock PlanningScene
        //   planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_ptr->getPlanningSceneMonitorNonConst());
        //   scene->processCollisionObjectMsg(object_to_attach);
        // }  // Unlock PlanningScene

        planning_scene_interface.applyCollisionObject(object_to_attach);

        RCLCPP_INFO(this->get_logger(), "add_object_callback ended");
        response->result = true;

      }


void MoveRobotServer::remove_object_callback(
      const std::shared_ptr<RemoveObject::Request> request,
      const std::shared_ptr<RemoveObject::Response> response) {
        std::vector<std::string> object_ids;
        object_ids.push_back(request->object_id);
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        //* Remove object until actually removed.
        const int max_attempts = 15;
        int attempts = 0;
        bool object_removed = false;

       //visual_tools->deleteAllMarkers();
      

        while (!object_removed) {
            auto object_result = planning_scene_interface.getObjects(object_ids);
            planning_scene_interface.removeCollisionObjects(object_ids);
            auto object_it = object_result.find(request->object_id);

            if (object_it == object_result.end()) {
                object_removed = true;
                response->result = true;
            } else if (attempts >= max_attempts) {
                response->result = false;
                break;
            }
            attempts++;
        }
        // TODO: add rm container.

        // planning_scene_interface.removeCollisionObjects(object_ids);
        // RCLCPP_INFO(this->get_logger(), "remove_object_callback ended");
        // response->result = true;
      }



void MoveRobotServer::attach_object_callback(
      const std::shared_ptr<AttachObject::Request> request,
      const std::shared_ptr<AttachObject::Response> response) {
        std::vector<std::string> touch_links;
        touch_links.push_back("panda_rightfinger");
        touch_links.push_back("panda_leftfinger");
        response->result = move_group_->attachObject(request->object_id, "panda_hand_tcp", touch_links);
        RCLCPP_INFO(this->get_logger(), "attach_object_callback ended");

      }

void MoveRobotServer::detach_object_callback(
      const std::shared_ptr<DetachObject::Request> request,
      const std::shared_ptr<DetachObject::Response> response) {
        response->result = move_group_->detachObject(request->object_id);
        RCLCPP_INFO(this->get_logger(), "detach_object_callback ended");
      }




bool MoveRobotServer::ArmMoveJ(const std_msgs::msg::Float64MultiArray & msg){
  std::vector<double> joints;
  for(long unsigned int i = 0; i<msg.data.size();i++)
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


bool MoveRobotServer::ArmMoveL(const geometry_msgs::msg::PoseStamped & msg){
  std::vector<geometry_msgs::msg::Pose> waypoints;
  move_group_->setPoseReferenceFrame(base_link);


  std::vector<geometry_msgs::msg::Pose> interpolated_poses;
  geometry_msgs::msg::PoseStamped pose1;
  pose1.pose = move_group_->getCurrentPose().pose;
  geometry_msgs::msg::PoseStamped pose2 = msg;
  double separation_distance = 0.005;
    // Calculate the linear interpolation between pose1 and pose2
    // based on the separation_distance

    // Calculate the direction vector from pose1 to pose2
    double dx = pose2.pose.position.x - pose1.pose.position.x;
    double dy = pose2.pose.position.y - pose1.pose.position.y;
    double dz = pose2.pose.position.z - pose1.pose.position.z;

    // Calculate the distance between pose1 and pose2
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

    // Calculate the number of interpolation points
    int num_points = static_cast<int>(std::ceil(distance / separation_distance));

    // Calculate the step size for each interpolation point
    // double step_size = distance / num_points;

    // Interpolate the points and add them to the vector
    for (int i = 0; i <= num_points; ++i)
    {
      double t = static_cast<double>(i) / num_points;

      geometry_msgs::msg::Pose interpolated_pose;
      interpolated_pose.position.x = pose1.pose.position.x + t * dx;
      interpolated_pose.position.y = pose1.pose.position.y + t * dy;
      interpolated_pose.position.z = pose1.pose.position.z + t * dz;
      interpolated_pose.orientation = pose2.pose.orientation;
      // Add the interpolated pose to the vector
      interpolated_poses.push_back(interpolated_pose);
    }



  waypoints.push_back(move_group_->getCurrentPose().pose);



  waypoints.push_back(msg.pose);

  moveit_msgs::msg::RobotTrajectory trajectory;
  // const double jump_threshold = 0.001;
  // const double eef_step = 0.001;
  // double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  move_group_->execute(trajectory);

  return true;
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
    RCLCPP_INFO(this->get_logger(), "set pose goal");
    bool success = static_cast<bool>(move_group_->plan(my_plan));
    RCLCPP_INFO(this->get_logger(), " (movement) %s", success ? "" : "FAILED");
    if(success == true){
        //move_group.move();
        // move_group_->setStartStateToCurrentState();
        RCLCPP_INFO(this->get_logger(), "about to exec");
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

    return false;


}

rclcpp_action::GoalResponse MoveRobotServer::home_arm_handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Home::Goal> goal)
  {
    (void)goal; //ignore unused parameter warning
    RCLCPP_INFO(this->get_logger(), "Received goal homing");
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
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group_->setNamedTarget("ready");
    move_group_->setPlanningPipelineId("ompl");
    move_group_->setStartStateToCurrentState();
    move_group_->clearPathConstraints();
    move_group_->setMaxVelocityScalingFactor(0.1);
    move_group_->setMaxAccelerationScalingFactor(0.1);
    auto feedback = std::make_shared<Home::Feedback>();
    auto result = std::make_shared<Home::Result>();

    bool success = static_cast<bool>(move_group_->plan(my_plan));
              RCLCPP_INFO(this->get_logger(), " (movement) %s", success ? "" : "FAILED");
              if(success == true){
                  //move_group.move();
                  // move_group_->setStartStateToCurrentState();
                  if(move_group_->execute(my_plan).val == 1){
                    result->done = true;
                    goal_handle->succeed(result);
                    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                  }
                  else{
                    move_group_->setStartStateToCurrentState();
                    move_group_->setNamedTarget("ready");
                    bool success = static_cast<bool>(move_group_->plan(my_plan));
                      RCLCPP_INFO(this->get_logger(), " (movement) %s", success ? "" : "FAILED");
                    if(success == true){
                      // move_group.move();
                      move_group_->setStartStateToCurrentState();
                      if(move_group_->execute(my_plan).val == 1){
                        result->done = true;
                        goal_handle->succeed(result);
                        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                      }
                      else{
                        result->done = false;
                        goal_handle->abort(result);
                        RCLCPP_INFO(this->get_logger(), "Goal arborted");
                      }

                    }

                  }
              }
              else{
                  result->done = false;
                  goal_handle->abort(result);
                  RCLCPP_INFO(this->get_logger(), "Goal arborted");
                }


  }


rclcpp_action::GoalResponse MoveRobotServer::gripper_joint_handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const GripperJoint::Goal> goal)
  {
    (void)goal; //ignore unused parameter warning
    RCLCPP_INFO(this->get_logger(), "Received goal requsdadest with sleep time %f",goal->joint_1);
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
        move_gripper_group_->setMaxAccelerationScalingFactor(goal->accel);
        move_gripper_group_->setMaxVelocityScalingFactor(goal->speed);
        RCLCPP_INFO(this->get_logger(), "opening %f",goal->joint_1);
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
    (void)goal; //ignore unused parameter warning
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
        move_gripper_group_->setMaxAccelerationScalingFactor(goal->accel);
        move_gripper_group_->setMaxVelocityScalingFactor(goal->speed);
        RCLCPP_INFO(this->get_logger(), "opening %d",goal->open);
        if(MoveRobotServer::MoveGripper(joints)){

          result->done = true;
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
        else{
          result->done = false;
          goal_handle->abort(result);
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
          goal_handle->abort(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
        }

      }
  }


////////// ARM ACTION SERVICES //////////////7


rclcpp_action::GoalResponse MoveRobotServer::arm_move_pliz_ptp_pose_msg_handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ArmMovePlizPtpPoseMsg::Goal> goal)
  {
    (void)goal; //ignore unused parameter warning
    // RCLCPP_INFO(this->get_logger(), "Received goal requsdadest with sleep time %d",goal->pose);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse MoveRobotServer::arm_move_pliz_ptp_pose_msg_handle_cancel(
    const std::shared_ptr<GoalHandleArmMovePlizPtpPoseMsg> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void MoveRobotServer::arm_move_pliz_ptp_pose_msg_handle_accepted(const std::shared_ptr<GoalHandleArmMovePlizPtpPoseMsg> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&MoveRobotServer::arm_move_pliz_ptp_pose_msg_execute, this, _1), goal_handle}.detach();

  }

  void MoveRobotServer::arm_move_pliz_ptp_pose_msg_execute(const std::shared_ptr<GoalHandleArmMovePlizPtpPoseMsg> goal_handle){
      auto result = std::make_shared<ArmMovePlizPtpPoseMsg::Result>();

      const auto goal = goal_handle->get_goal();
      auto pose = goal->pose;
      move_group_->clearPathConstraints();

      move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
      move_group_->setPlannerId("PTP");
      move_group_->setMaxAccelerationScalingFactor(goal->accel);
      move_group_->setMaxVelocityScalingFactor(goal->speed);

      if(MoveRobotServer::Move(pose))
      {
          result->done = true;
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      }
      else{
          result->done = false;
          goal_handle->abort(result);
          RCLCPP_INFO(this->get_logger(), "Goal abort");
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
      //     goal_handle->succeed(result);ngPipelineId("pilz_industrial_motion_planner");
      // move_group_->setPlannerId("LIN");
      //     RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      //   }
      //   else{
      //     result->done = false;
      //     goal_handle->canceled(result);
      //     RCLCPP_INFO(this->get_logger(), "Goal canceled");
      //   }

      // }
  }


//////(/)

rclcpp_action::GoalResponse MoveRobotServer::arm_move_pliz_lin_pose_msg_handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ArmMovePlizLinPoseMsg::Goal> goal)
  {
    (void)goal; //ignore unused parameter warning
    // RCLCPP_INFO(this->get_logger(), "Received goal requsdadest with sleep time %d",goal->pose);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse MoveRobotServer::arm_move_pliz_lin_pose_msg_handle_cancel(
    const std::shared_ptr<GoalHandleArmMovePlizLinPoseMsg> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void MoveRobotServer::arm_move_pliz_lin_pose_msg_handle_accepted(const std::shared_ptr<GoalHandleArmMovePlizLinPoseMsg> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&MoveRobotServer::arm_move_pliz_lin_pose_msg_execute, this, _1), goal_handle}.detach();

  }

  void MoveRobotServer::arm_move_pliz_lin_pose_msg_execute(const std::shared_ptr<GoalHandleArmMovePlizLinPoseMsg> goal_handle){
      auto result = std::make_shared<ArmMovePlizLinPoseMsg::Result>();

      const auto goal = goal_handle->get_goal();
      auto pose = goal->pose;

      move_group_->clearPathConstraints();

      // move_group_->setPlanningPipelineId("ompl");
      // RCLCPP_INFO(this->get_logger(), "Planned position x: %f, y: %f, z: %f", pose.pose.position.x, pose.pose.position.y,
      //         pose.pose.position.z);
      // // move_group_->setPlannerId("LIN");
      // move_group_->setMaxAccelerationScalingFactor(goal->accel);
      // move_group_->setMaxVelocityScalingFactor(goal->speed);
      // if(MoveRobotServer::ArmMoveL(pose)){
      //                   result->done = true;
      //                   goal_handle->succeed(result);
      //                   RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      // }




      move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
      move_group_->setPlannerId("LIN");
      move_group_->setMaxAccelerationScalingFactor(goal->accel);
      move_group_->setMaxVelocityScalingFactor(goal->speed);
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // RCLCPP_INFO(this->get_logger(), "Planned position x: %f, y: %f, z: %f", pose.position.x, pose_goal.position.y,
    //           pose_goal.position.z);
    // bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // RCLCPP_INFO(this->get_logger(), " (movement) %s", success ? "" : "FAILED");
    // move_group_->move();
    // pose.header.frame_id = "panda1_link0";

    move_group_->setStartStateToCurrentState();
    // move_group_->setPoseReferenceFrame("");
    move_group_->setPoseReferenceFrame(base_link);
    // pose.header.frame_id = "";
    move_group_->setPoseTarget(pose);
    bool success = static_cast<bool>(move_group_->plan(my_plan));
              RCLCPP_INFO(this->get_logger(), " (movement) %s", success ? "" : "FAILED");
              if(success == true){
                  //move_group.move();
                  // move_group_->setStartStateToCurrentState();
                  if(move_group_->execute(my_plan).val == 1){
                    result->done = true;
                    goal_handle->succeed(result);
                    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                  }
                  else{
                    move_group_->setStartStateToCurrentState();
                    move_group_->setPoseTarget(pose.pose);
                    bool success = static_cast<bool>(move_group_->plan(my_plan));
                      RCLCPP_INFO(this->get_logger(), " (movement) %s", success ? "" : "FAILED");
                    if(success == true){
                      // move_group.move();
                      move_group_->setStartStateToCurrentState();
                      if(move_group_->execute(my_plan).val == 1){
                        result->done = true;
                        goal_handle->succeed(result);
                        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                      }
                      else{
                        result->done = false;
                        goal_handle->abort(result);
                        RCLCPP_INFO(this->get_logger(), "Goal canceled");
                      }

                    }

                  }
              }
              else{
                  result->done = false;
                  goal_handle->abort(result);
                  RCLCPP_INFO(this->get_logger(), "Goal canceled");
                }


  }


/////

rclcpp_action::GoalResponse MoveRobotServer::arm_move_pose_msg_handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ArmMovePoseMsg::Goal> goal)
  {
    (void)goal; //ignore unused parameter warning
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
      bool keep_orientation = goal->keep_orientation;
      auto pose = goal->pose;
      move_group_->setPlanningPipelineId("ompl");
      move_group_->setMaxAccelerationScalingFactor(goal->accel);
      move_group_->setMaxVelocityScalingFactor(goal->speed);


      // TODO: Add constraint for holding side of tube
      if(keep_orientation){
        move_group_->setPlannerId("geometric::RRTstar");
        moveit_msgs::msg::OrientationConstraint orientation_constraint;
        orientation_constraint.header.frame_id = move_group_ ->getPoseReferenceFrame();
        orientation_constraint.link_name = move_group_ ->getEndEffectorLink();

        orientation_constraint.orientation = goal->pose.pose.orientation;
        orientation_constraint.absolute_x_axis_tolerance = 0.4;
        orientation_constraint.absolute_y_axis_tolerance = 0.4;
        orientation_constraint.absolute_z_axis_tolerance = 1.57;
        orientation_constraint.weight = 1.0;

        moveit_msgs::msg::Constraints orientation_constraints;
        orientation_constraints.orientation_constraints.emplace_back(orientation_constraint);

        move_group_->setPathConstraints(orientation_constraints);

        move_group_->setPlanningTime(20.0);
      }
      else if(goal->lin){
        move_group_->setPlannerId("geometric::RRTstar");
        moveit_msgs::msg::OrientationConstraint orientation_constraint;
        orientation_constraint.header.frame_id = move_group_ ->getPoseReferenceFrame();
        orientation_constraint.link_name = move_group_ ->getEndEffectorLink();

        orientation_constraint.orientation = goal->pose.pose.orientation;
        orientation_constraint.absolute_x_axis_tolerance = 1.57;
        orientation_constraint.absolute_y_axis_tolerance = 0.4;
        orientation_constraint.absolute_z_axis_tolerance = 0.4;
        orientation_constraint.weight = 1.0;

        moveit_msgs::msg::Constraints orientation_constraints;
        orientation_constraints.orientation_constraints.emplace_back(orientation_constraint);

        move_group_->setPathConstraints(orientation_constraints);

        move_group_->setPlanningTime(20.0);
        // move_group_->setPlanningTime(20.0);
        // // LINE CONSTRAINTS
        // moveit_msgs::msg::PositionConstraint line_constraint;
        // line_constraint.header.frame_id = move_group_->getPoseReferenceFrame();
        // line_constraint.link_name = move_group_->getEndEffectorLink();
        // shape_msgs::msg::SolidPrimitive line;
        // line.type = shape_msgs::msg::SolidPrimitive::BOX;
        // line.dimensions = { 0.0005, 0.0005, 1.0 };
        // line_constraint.constraint_region.primitives.emplace_back(line);

        // geometry_msgs::msg::Pose line_pose;
        // line_pose.position.x = goal->pose.pose.position.x;
        // line_pose.position.y = goal->pose.pose.position.y;
        // line_pose.position.z = goal->pose.pose.position.z;
        // line_pose.orientation.x = goal->pose.pose.orientation.x;
        // line_pose.orientation.y = goal->pose.pose.orientation.x;
        // line_pose.orientation.z = goal->pose.pose.orientation.x;
        // line_pose.orientation.w = goal->pose.pose.orientation.x;
        // line_constraint.constraint_region.primitive_poses.emplace_back(line_pose);
        // line_constraint.weight = 1.0;


        // // ORIENTATION CONSTRAINTS
        // moveit_msgs::msg::OrientationConstraint orientation_constraint;
        // orientation_constraint.header.frame_id = move_group_ ->getPoseReferenceFrame();
        // orientation_constraint.link_name = move_group_ ->getEndEffectorLink();

        // orientation_constraint.orientation = goal->pose.pose.orientation;
        // orientation_constraint.absolute_x_axis_tolerance = 0.2;
        // orientation_constraint.absolute_y_axis_tolerance = 0.2;
        // // orientation_constraint.absolute_z_axis_tolerance = 0.2;
        // orientation_constraint.weight = 1.0;


        // moveit_msgs::msg::Constraints line_constraints;
        // line_constraints.position_constraints.emplace_back(line_constraint);
        // line_constraints.orientation_constraints.emplace_back(orientation_constraint);

        // line_constraints.name = "use_equality_constraints";
        // move_group_->setPathConstraints(line_constraints);
      }
      else{
        // moveit_msgs::msg::OrientationConstraint orientation_constraint;
        // orientation_constraint.header.frame_id = move_group_ ->getPoseReferenceFrame();
        // orientation_constraint.link_name = move_group_ ->getEndEffectorLink();

        // orientation_constraint.orientation = goal->pose.pose.orientation;
        // orientation_constraint.absolute_x_axis_tolerance = 6.28;
        // orientation_constraint.absolute_y_axis_tolerance = 6.28;
        // orientation_constraint.absolute_z_axis_tolerance = 6.28;
        // orientation_constraint.weight = 0.0;

        // moveit_msgs::msg::Constraints orientation_constraints;
        // orientation_constraints.orientation_constraints.emplace_back(orientation_constraint);

        // move_group_->setPathConstraints(orientation_constraints);
        moveit_msgs::msg::Constraints empty_constraints;
        move_group_->setPathConstraints(empty_constraints);
        move_group_->clearPathConstraints();
        move_group_->setPlanningTime(10.0);

      }

      move_group_->setPoseTarget(goal->pose);
      // move_group_->plan(plan);

      if(goal->pose.header.frame_id != ""){
        move_group_->setPoseReferenceFrame(goal->pose.header.frame_id);
      }
      else{
        move_group_->setPoseReferenceFrame(base_link);
      }

      if(MoveRobotServer::Move(pose))
      {
          result->done = true;
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      }
      else{
          result->done = false;
          goal_handle->abort(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
      }

  }



rclcpp_action::GoalResponse MoveRobotServer::arm_move_pose_msg_tcp_handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ArmMovePoseMsgTcp::Goal> goal)
  {
    (void)goal; //ignore unused parameter warning
    // RCLCPP_INFO(this->get_logger(), "Received goal requsdadest with sleep time %d",goal->pose);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse MoveRobotServer::arm_move_pose_msg_tcp_handle_cancel(
    const std::shared_ptr<GoalHandleArmMovePoseMsgTcp> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void MoveRobotServer::arm_move_pose_msg_tcp_handle_accepted(const std::shared_ptr<GoalHandleArmMovePoseMsgTcp> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&MoveRobotServer::arm_move_pose_msg_tcp_execute, this, _1), goal_handle}.detach();

  }

  void MoveRobotServer::arm_move_pose_msg_tcp_execute(const std::shared_ptr<GoalHandleArmMovePoseMsgTcp> goal_handle){
      auto result = std::make_shared<ArmMovePoseMsgTcp::Result>();

      const auto goal = goal_handle->get_goal();
      auto pose = goal->pose;
      move_group_->setPlanningPipelineId("ompl");
      move_group_->setMaxAccelerationScalingFactor(goal->accel);
      move_group_->setMaxVelocityScalingFactor(goal->speed);
      move_group_->setEndEffectorLink(goal->tcp_frame);
      // move_group_->setMaxAccelerationScalingFactor(0.6);
      // move_group_->setMaxVelocityScalingFactor(0.6);
      moveit_msgs::msg::JointConstraint jcm;
      jcm.joint_name = "panda_joint2";
      jcm.position = 0.0;
      jcm.tolerance_below = -1.7028;
      jcm.tolerance_above = 1.7028;
      if(goal->pose.header.frame_id != ""){
        move_group_->setPoseReferenceFrame(goal->pose.header.frame_id);
      }
      else{
        move_group_->setPoseReferenceFrame(base_link);
      }

      if(MoveRobotServer::Move(pose))
      {
          move_group_->setEndEffectorLink(tcp_frame);
          result->done = true;
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      }
      else{
          move_group_->setEndEffectorLink(tcp_frame);
          result->done = false;
          goal_handle->abort(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
      }

  }


rclcpp_action::GoalResponse MoveRobotServer::arm_move_pose_handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ArmMovePose::Goal> goal)
  {
    (void)goal; //ignore unused parameter warning
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

      bool keep_orientation = goal->keep_orientation;



      // move_group_->setPoseTarget(goal->pose);
      // move_group_->setPlanningTime(10.0);
    // while (ss.good()) {
    //     std::string substr;
    //     getline(ss, substr, ',');
    //     v.push_back(substr);
    // }
    for(long unsigned int i = 0; i < pose.size(); i++)
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

      if(keep_orientation){
        RCLCPP_INFO(this->get_logger(), "keep orientation true");
        moveit_msgs::msg::OrientationConstraint orientation_constraint;
        orientation_constraint.header.frame_id = move_group_ ->getPoseReferenceFrame();
        orientation_constraint.link_name = move_group_ ->getEndEffectorLink();

        orientation_constraint.orientation.x = q_new.x();
        orientation_constraint.orientation.y = q_new.y();
        orientation_constraint.orientation.z = q_new.z();
        orientation_constraint.orientation.w = q_new.w();
        orientation_constraint.absolute_x_axis_tolerance = 0.2;
        orientation_constraint.absolute_y_axis_tolerance = 0.2;
        orientation_constraint.absolute_z_axis_tolerance = 1.57;
        orientation_constraint.weight = 1.0;

        moveit_msgs::msg::Constraints orientation_constraints;
        orientation_constraints.orientation_constraints.emplace_back(orientation_constraint);

        move_group_->setPathConstraints(orientation_constraints);
        move_group_->setPlanningTime(20.0);
      }
      else{
        // moveit_msgs::msg::OrientationConstraint orientation_constraint;
        // orientation_constraint.header.frame_id = move_group_ ->getPoseReferenceFrame();
        // orientation_constraint.link_name = move_group_ ->getEndEffectorLink();

        // orientation_constraint.orientation.x = pos[3];
        // orientation_constraint.orientation.y = pos[4];
        // orientation_constraint.orientation.z = pos[5];
        // orientation_constraint.orientation.w = pos[6];
        // orientation_constraint.absolute_x_axis_tolerance = 6.28;
        // orientation_constraint.absolute_y_axis_tolerance = 6.28;
        // orientation_constraint.absolute_z_axis_tolerance = 6.28;
        // orientation_constraint.weight = 0.1;

        // moveit_msgs::msg::Constraints orientation_constraints;
        // orientation_constraints.orientation_constraints.emplace_back(orientation_constraint);

        // move_group_->setPathConstraints(orientation_constraints);
        moveit_msgs::msg::Constraints empty_constraints;
        move_group_->setPathConstraints(empty_constraints);
        move_group_->clearPathConstraints();

      }
      move_group_->setPlanningPipelineId("ompl");
      move_group_->setMaxAccelerationScalingFactor(goal->accel);
      move_group_->setMaxVelocityScalingFactor(goal->speed);
      // move_group_->setMaxAccelerationScalingFactor(0.6);
      // move_group_->setMaxVelocityScalingFactor(0.6);
      if(MoveRobotServer::Move(pose_goal))
      {
          result->done = true;
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      }
      else{
          result->done = false;
          goal_handle->abort(result);
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
    (void)goal; //ignore unused parameter warning
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
      for(long unsigned int i = 0; i<joint_pose.size();i++)
      {
        joints_.data.push_back(joint_pose[i]);
       }

      move_group_->setPlanningPipelineId("ompl");
      move_group_->setMaxAccelerationScalingFactor(goal->accel);
      move_group_->setMaxVelocityScalingFactor(goal->speed);
      // move_group_->setMaxAccelerationScalingFactor(0.6);
      // move_group_->setMaxVelocityScalingFactor(0.6);

      if(MoveRobotServer::ArmMoveJ(joints_))
      {
          result->done = true;
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      }
      else{
          result->done = false;
          goal_handle->abort(result);
          RCLCPP_INFO(this->get_logger(), "Goal abort");
      }
  }

rclcpp_action::GoalResponse MoveRobotServer::arm_move_joints_relative_handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ArmMoveJointsRelative::Goal> goal)
  {
    (void)goal; //ignore unused parameter warning
    // RCLCPP_INFO(this->get_logger(), "Received goal requsdadest with sleep time %d",goal->pose);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse MoveRobotServer::arm_move_joints_relative_handle_cancel(
    const std::shared_ptr<GoalHandleArmMoveJointsRelative> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void MoveRobotServer::arm_move_joints_relative_handle_accepted(const std::shared_ptr<GoalHandleArmMoveJointsRelative> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&MoveRobotServer::arm_move_joints_relative_execute, this, _1), goal_handle}.detach();

  }

  void MoveRobotServer::arm_move_joints_relative_execute(const std::shared_ptr<GoalHandleArmMoveJointsRelative> goal_handle){
      auto result = std::make_shared<ArmMoveJointsRelative::Result>();

      move_group_->clearPathConstraints();

      std_msgs::msg::Float64MultiArray joints_;
      const auto goal = goal_handle->get_goal();
      auto joint_pose = goal->joints;
      for(long unsigned int i = 0; i<joint_pose.size();i++)
      {
        joints_.data.push_back(joint_pose[i]);
       }
      auto joint_model_group = move_group_->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
      auto current_state = move_group_->getCurrentState(10);
      std::vector<double> joint_group_positions;
      current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

      for(long unsigned int j = 0; j < joints_.data.size();j++){
        joints_.data[j] += joint_group_positions[j];
      }

      move_group_->setPlanningPipelineId("ompl");
      move_group_->setMaxAccelerationScalingFactor(goal->accel);
      move_group_->setMaxVelocityScalingFactor(goal->speed);
      // move_group_->setMaxAccelerationScalingFactor(0.6);
      // move_group_->setMaxVelocityScalingFactor(0.6);

      if(MoveRobotServer::ArmMoveJ(joints_))
      {
          result->done = true;
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      }
      else{
          result->done = false;
          goal_handle->abort(result);
          RCLCPP_INFO(this->get_logger(), "Goal abort");
      }
  }



//////////  SLEEP ACTION //////////////

rclcpp_action::GoalResponse MoveRobotServer::sleep_handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Sleep::Goal> goal)
  {
    (void)goal; //ignore unused parameter warning
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


// TODO: Move below to header file
  // check the CollisionObject types this stage can handle
  inline bool isValidObject(const moveit_msgs::msg::CollisionObject &o) {
    return (o.meshes.size() == 1 && o.mesh_poses.size() == 1 &&
            o.primitives.empty()) ||
          (o.meshes.empty() && o.primitives.size() == 1 &&
            o.primitive_poses.size() == 1 &&
            o.primitives[0].type == shape_msgs::msg::SolidPrimitive::CYLINDER);
  }

  /* compute height of the CollisionObject
    This is only useful for meshes, when they are centered */
  inline double getObjectHeight(const moveit_msgs::msg::CollisionObject &o) {
    if (!o.meshes.empty()) {
      double x, y, z;
      geometric_shapes::getShapeExtents(o.meshes[0], x, y, z);
      return z;
    } else {
      // validations guarantees this is a cylinder
      return o.primitives[0]
          .dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT];
    }
  }
// TODO: Move above to header file

// TODO: [ ] Testing
//! Get pre-pour pose
void MoveRobotServer::get_pre_pour_pose_callback(
      const std::shared_ptr<GetPrePourPose::Request> request,
      const std::shared_ptr<GetPrePourPose::Response> response){
        RCLCPP_INFO(this->get_logger(), "get_pre_pour_pose_callback called");

        move_group_->setMaxAccelerationScalingFactor(0.015);
        move_group_->setMaxVelocityScalingFactor(0.015);

        auto container_name = request->container_name; //object_id
        auto bottle_name = request->bottle_name;

        // const std::string container_name = std::to_string(request->object_id);

        RCLCPP_INFO(this->get_logger(), bottle_name.c_str());

        std::vector<std::string> object_ids;
        object_ids.push_back(bottle_name);

        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


        // ? Get the poses from the objects identified by the given object ids list
        // ? Assert that the object id is in the list of objects and is a valid object

        // Get the poses of the objects
        std::map<std::string, geometry_msgs::msg::Pose> object_poses = planning_scene_interface.getObjectPoses({bottle_name});

        // Check if the pose of the "bottle" object was found
        if(object_poses.find(bottle_name) != object_poses.end()) {
            // Get the pose of the "bottle" object
            geometry_msgs::msg::Pose bottle_pose = object_poses[bottle_name];
            ////visual_tools->publishAxisLabeled(bottle_pose, bottle_name);
            ////visual_tools->trigger();
            

            // Set the target pose
            bottle_pose.position.x -= 0.005;
            bottle_pose.position.y += 0.010;
            bottle_pose.position.z += 0.0;
            // Get the orientation of the bottle
            tf2::Quaternion quat(
                bottle_pose.orientation.x,
                bottle_pose.orientation.y,
                bottle_pose.orientation.z,
                bottle_pose.orientation.w
            );

            tf2::Quaternion quat_y_rotation;
            quat_y_rotation.setRPY(0, M_PI/2, 0);

            tf2::Quaternion quat_z_rotation;
            quat_z_rotation.setRPY(0, 0, M_PI);

            // Combine the rotations
            quat = quat * quat_y_rotation; //* quat_z_rotation;

            // Update the orientation of the bottle
            bottle_pose.orientation.x = quat.x();
            bottle_pose.orientation.y = quat.y();
            bottle_pose.orientation.z = quat.z();
            bottle_pose.orientation.w = quat.w();

           //visual_tools->publishAxisLabeled(bottle_pose, "grasp_target");
            //visual_tools->trigger();

            // move_group.setPoseTarget(bottle_pose);

            // tf2::Quaternion quat_x_rotation;
            // quat_x_rotation.setRPY(M_PI/12, 0, 0); // M_PI/12 radians = 15 degrees

            geometry_msgs::msg::PoseStamped stamped_pose1;
            stamped_pose1.pose = bottle_pose;
            stamped_pose1.header.frame_id = "panda_link0";
            stamped_pose1.header.stamp = rclcpp::Clock().now();
            response->result = true;
            response->pose_bottle = stamped_pose1;

              std::map<std::string, geometry_msgs::msg::Pose> container_poses_ = planning_scene_interface.getObjectPoses({container_name});

              if(container_poses_.find(container_name) != container_poses_.end()) {
                geometry_msgs::msg::Pose container_pose_ = container_poses_[container_name];


                //visual_tools->publishAxisLabeled(container_pose_, "container_pose");
                //visual_tools->trigger();

                container_pose_.position.x -= 0.03;
                container_pose_.position.y -= 0.08;
                container_pose_.position.z += 0.11;

                tf2::Quaternion quat2(
                    container_pose_.orientation.x,
                    container_pose_.orientation.y,
                    container_pose_.orientation.z,
                    container_pose_.orientation.w
                );

                // Combine the rotations
                quat2 = quat2 * quat_y_rotation;// * quat_z_rotation;

                // Update the orientation of the bottle
                container_pose_.orientation.x = quat.x();
                container_pose_.orientation.y = quat.y();
                container_pose_.orientation.z = quat.z();
                container_pose_.orientation.w = quat.w();

                geometry_msgs::msg::PoseStamped stamped_pose2;
                stamped_pose2.pose = container_pose_;
                stamped_pose2.header.frame_id = "panda_link0";
                stamped_pose2.header.stamp = rclcpp::Clock().now();
                response->result = true;
                response->pose_container = stamped_pose2;

                //visual_tools->publishAxisLabeled(container_pose_, "container_pose_above");
                //visual_tools->trigger();


          }

        }
            RCLCPP_INFO(this->get_logger(), "get_pre_pour_pose_callback ended");
  }
    

//! get_pre_pour_pose END

rclcpp_action::GoalResponse MoveRobotServer::arm_move_trajectory_pour_handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ArmMoveTrajectoryPour::Goal> goal)
  {
    (void)goal; //ignore unused parameter warning
    // RCLCPP_INFO(this->get_logger(), "Received goal requsdadest with sleep time %d",goal->pose);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse MoveRobotServer::arm_move_trajectory_pour_handle_cancel(
    const std::shared_ptr<GoalHandleArmMoveTrajectoryPour> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void MoveRobotServer::arm_move_trajectory_pour_handle_accepted(const std::shared_ptr<GoalHandleArmMoveTrajectoryPour> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&MoveRobotServer::arm_move_trajectory_pour_execute, this, _1), goal_handle}.detach();

  }

  // ! ------------------------ PourInto ------------------------

  void computePouringWaypoints(const Eigen::Isometry3d &start_tip_pose,
                              double tilt_angle,
                              const Eigen::Translation3d &pouring_offset,
                              EigenSTL::vector_Isometry3d &waypoints,
                              unsigned long nr_of_waypoints = 10) {
    Eigen::Isometry3d start_tip_rotation(start_tip_pose);
    start_tip_rotation.translation().fill(0);

    waypoints.push_back(start_tip_pose);

    for (unsigned int i = 1; i <= nr_of_waypoints; ++i) {
      const double fraction = static_cast<double>(i) / nr_of_waypoints;
      const double exp_fraction = fraction * fraction;
      const double offset_fraction =
          std::pow(fraction, 1.0 / 5.0) +
          (-4.5 * fraction * fraction +
          4.5 * fraction); // custom trajectory translating away from cup center

      // linear interpolation for tilt angle
      Eigen::Isometry3d rotation =
          Eigen::AngleAxisd(fraction * tilt_angle, Eigen::Vector3d::UnitX()) *
          start_tip_rotation;

      // exponential interpolation towards container rim + offset
      Eigen::Translation3d translation(
          start_tip_pose.translation() * (1 - exp_fraction) +
          pouring_offset.translation() * exp_fraction);

      translation.y() = start_tip_pose.translation().y() * (1 - offset_fraction) +
                        pouring_offset.translation().y() * (offset_fraction);

      waypoints.push_back(translation * rotation);
    }
  }

  void MoveRobotServer::arm_move_trajectory_pour_execute(const std::shared_ptr<GoalHandleArmMoveTrajectoryPour> goal_handle){
      auto result = std::make_shared<ArmMoveTrajectoryPour::Result>();

      RCLCPP_WARN(this->get_logger(), "IN POUR");

      // Variables
      const auto goal = goal_handle->get_goal();
      // int8_t container_name_i = goal->container_name;
      // int8_t bottle_name_i = goal->bottle_name;
      double tilt_angle = goal->tilt_angle;
      double min_path_fraction = goal->min_path_fraction;
      auto container_name = goal->container_name;
      auto bottle_name = goal->bottle_name;
      // auto container_name = std::to_string(container_name_i);
      // auto bottle_name = std::to_string(bottle_name_i);

      const Eigen::Translation3d pour_offset(Eigen::Vector3d(0.00, -0.04, 0.05));

      geometry_msgs::msg::Vector3Stamped pouring_axis;
      pouring_axis.header.frame_id = container_name;
      pouring_axis.vector.x = 1.0;

      size_t waypoint_count = 10;

      //! ---------- Get container object

      std::vector<std::string> object_ids_;
      object_ids_.push_back(container_name);

      moveit_msgs::msg::CollisionObject container_object;
      auto object_result = planning_scene_interface.getObjects(object_ids_);

      auto object_it = object_result.find(container_name);

      if (object_it == object_result.end()) {
          throw std::runtime_error("Object ID '" + container_name + "' not found");
      }

      // std::string id = object_it->second.id;
      geometry_msgs::msg::Pose container_pose_ = object_it->second.pose;
      // object_recognition_msgs::msg::ObjectType type = object_it->second.type;

      container_object = object_it->second;

      // const Eigen::Translation3d pour_offset(Eigen::Vector3d(0.03, -0.03, 0.0));


      if (!isValidObject(container_object))
        throw std::runtime_error(
            "PourInto: container is neither a valid cylinder nor mesh.");

      // ! ---------- Get container pose (frame)

      std::map<std::string, geometry_msgs::msg::Pose> container_poses_ = planning_scene_interface.getObjectPoses({"container"});

      if(container_poses_.find("container") != container_poses_.end()) {
        geometry_msgs::msg::Pose container_pose_ = container_poses_["container"];

      // Get the orientation of the bottle
      tf2::Quaternion quat(
          container_pose_.orientation.x,
          container_pose_.orientation.y,
          container_pose_.orientation.z,
          container_pose_.orientation.w
      );
      }

      //! ----------- Get bottle object

      std::vector<std::string> attached_object_ids;
      attached_object_ids.push_back(bottle_name);

      moveit_msgs::msg::AttachedCollisionObject bottle;
      moveit_msgs::msg::CollisionObject bottle_object;

      auto attached_object_result = planning_scene_interface.getAttachedObjects(attached_object_ids);

      auto attached_object_it = attached_object_result.find(bottle_name);

      if (attached_object_it == attached_object_result.end()) {
          throw std::runtime_error("Object ID '" + container_name + "' not found");
      }

      // AttachedObject -> Collisionobject
      bottle_object = attached_object_it->second.object;

      if (!isValidObject(bottle_object))
        throw std::runtime_error(
            "PourInto: container is neither a valid cylinder nor mesh.");


      geometry_msgs::msg::Pose bottle_pose_ = bottle_object.pose;

      //! We got objects and poses, do thing with them.

      //? container frame:
      // - top-center of container object
      // - rotation should coincide with the planning frame
      // Create an Eigen Isometry3d for the output
      Eigen::Isometry3d container_frame_n;
      tf2::fromMsg(container_pose_, container_frame_n);
      container_frame_n = container_frame_n * Eigen::Translation3d(Eigen::Vector3d(0, 0, -container_object.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] / 2 + 0.011));
      container_frame_n.linear().setIdentity();

      //visual_tools->publishAxisLabeled(container_frame_n, "CONTAINER_FRAME");
      //visual_tools->trigger();

      //! TESTING

      // Get the current pose of the end effector
      geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose();

      Eigen::Isometry3d bottle_tip_frame_n_id;
      tf2::fromMsg(current_pose.pose, bottle_tip_frame_n_id);

      bottle_tip_frame_n_id.linear().setIdentity(); //dsc rot
      Eigen::Isometry3d bottle_tip_frame_n;
            Eigen::Isometry3d bottle_tip_frame_n_extra;
      Eigen::Isometry3d bottle_tip_frame_n_minus;

      Eigen::Isometry3d tool_link_in_world_frame;
      tf2::fromMsg(current_pose.pose, tool_link_in_world_frame);

      bottle_tip_frame_n = bottle_tip_frame_n_id * Eigen::Translation3d(Eigen::Vector3d(0, 0, bottle_object.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT]));

      Eigen::Isometry3d bottle_tip_in_tool_link;
      bottle_tip_in_tool_link = tool_link_in_world_frame.inverse() * bottle_tip_frame_n;

      Eigen::Isometry3d bottle_tip_in_container_frame;
      bottle_tip_in_container_frame = container_frame_n.inverse() * bottle_tip_frame_n;

      // Convert geometry_msgs::msg::Pose to Eigen::Isometry3d
      Eigen::Isometry3d container_pose_eigen;
      tf2::fromMsg(container_pose_, container_pose_eigen);

      Eigen::Isometry3d bottle_tip_frame_n_id_eigen;
      tf2::fromMsg(current_pose.pose, bottle_tip_frame_n_id_eigen);

      // visual_tools.publishAxisLabeled(bottle_tip_in_tool_link, "BOTTLE_TIP_IN_TOOL_LINK!");
      // visual_tools.trigger();

      // geometry_msgs::msg::Vector3Stamped pouring_axis;
      // pouring_axis.header.frame_id = container_name;
      // pouring_axis.vector.x = 1.0; // should be ok, minus if hand is oriented differently

      // Convert the Vector3Stamped message to an Eigen Vector3d
      Eigen::Vector3d tilt_axis;
      tf2::fromMsg(pouring_axis.vector, tilt_axis);

      //Transform tilt_axis from the frame of pouring_axis to the frame of container_frame
      tilt_axis = container_frame_n.inverse()  * tilt_axis; //* container_frame_n

      // Always tilt around axis in x-y plane
      tilt_axis.z() = 0.0;

      // Calculate the tilt axis angle
      double tilt_axis_angle = std::atan2(tilt_axis.y(), tilt_axis.x());

      /* Cartesian waypoints for pouring motion */
      EigenSTL::vector_Isometry3d waypoints;

      /* generate waypoints in y-z plane */
      computePouringWaypoints(bottle_tip_in_container_frame, tilt_angle,
                              pour_offset, waypoints,
                              waypoint_count);

      /* rotate y-z plane so tilt motion is along the specified tilt_axis */
      for (auto &waypoint : waypoints)
        waypoint = Eigen::AngleAxisd(tilt_axis_angle, Eigen::Vector3d::UnitZ()) *
                  waypoint *
                  Eigen::AngleAxisd(-tilt_axis_angle, Eigen::Vector3d::UnitZ());

      /* transform waypoints to planning frame */
      for (auto &waypoint : waypoints)
        waypoint = container_frame_n * waypoint;


      for (auto &waypoint : waypoints) {
        geometry_msgs::msg::PoseStamped p;
        p.header.frame_id = move_group_->getPlanningFrame();
        std::cout << p.header.frame_id << std::endl;
        p.pose = tf2::toMsg(waypoint);
      }

        /* specify waypoints for tool link, not for bottle tip */
      for (auto &waypoint : waypoints)
      waypoint = waypoint * bottle_tip_in_tool_link.inverse();

      // std::vector<moveit::core::RobotStatePtr> traj;

      const double jump_threshold = 0.0;
      const double eef_step = 0.03;

      moveit_msgs::msg::RobotTrajectory trajectory;
      std::vector<geometry_msgs::msg::Pose> pose_waypoints;
      // Get the current pose and add it as the first waypoint
      geometry_msgs::msg::Pose current_pose_now = move_group_->getCurrentPose().pose;
      geometry_msgs::msg::Pose converted_pose;
      tf2::convert(current_pose_now, converted_pose);
      pose_waypoints.push_back(converted_pose);

      for (const auto& waypoint : waypoints)
      {
          geometry_msgs::msg::Pose pose;
          tf2::convert(waypoint, pose);
          pose_waypoints.push_back(pose);
      }

      // bool path_ok = false;

      // double path_fraction;

      // int attempts = 0;


      // while (path_ok == false && attempts < 15) {
      //     double path_fraction = move_group_->computeCartesianPath(pose_waypoints, eef_step, jump_threshold, trajectory);

      //     if (path_fraction < min_path_fraction) {
      //         RCLCPP_WARN_STREAM(rclcpp::get_logger("pouring_planner"), "PourInto only produced motion for "
      //                             << path_fraction << " of the way. Rendering invalid");

      //         // move_group_->setEndEffectorLink(tcp_frame);
      //     } else {
      //         path_ok = true;
      //     }
      //     attempts++;
      // }
      double path_fraction = move_group_->computeCartesianPath(pose_waypoints, eef_step, jump_threshold, trajectory);


      Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();


      //       // Convert the waypoints to RobotState objects and add them to the traj vector
      // for (const auto& pose : pose_waypoints)
      // {
      //     auto state = std::make_shared<moveit::core::RobotState>(*current_state);
      //     state->setFromIK(move_group.getCurrentState()->getJointModelGroup(move_group.getName()), pose);
      //     traj.push_back(state);
      // }


      //visual_tools->publishText(text_pose, "Cartesian_Path", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
      //visual_tools->publishPath(waypoints, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
      // for (std::size_t i = 0; i < waypoints.size(); ++i)
      //visual_tools->publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rviz_visual_tools::SMALL);
      //visual_tools->trigger();


      // Get the current robot model
      auto robot_model = move_group_->getCurrentState()->getRobotModel();
      std::string group = move_group_->getName();

      // Create a robot_trajectory::RobotTrajectory
      auto robot_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, group);

      // Set the trajectory message
      robot_trajectory->setRobotTrajectoryMsg(*move_group_->getCurrentState(), trajectory);

      // Create a reverse trajectory
      auto reverse_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(*robot_trajectory);
      reverse_trajectory->reverse();

      // Create a moveit_msgs::msg::RobotTrajectory for the reverse trajectory
      moveit_msgs::msg::RobotTrajectory trajectory_msg;
      moveit_msgs::msg::RobotTrajectory reverse_trajectory_msg;

      // Get the reverse trajectory message
      reverse_trajectory->getRobotTrajectoryMsg(reverse_trajectory_msg);
      robot_trajectory->getRobotTrajectoryMsg(trajectory_msg);


       // Not good traj
      if (path_fraction < min_path_fraction) {
          RCLCPP_WARN_STREAM(rclcpp::get_logger("pouring_planner"), "PourInto only produced motion for "
                              << path_fraction << " of the way. Rendering invalid");

          // move_group_->setEndEffectorLink(tcp_frame);
          result->done = false;
          goal_handle->abort(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");

      } // else (good traj)

      RCLCPP_INFO(rclcpp::get_logger("pouring_planner"), "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", path_fraction * 100.0);

      //! At runtime, uncomment this.
      move_group_->execute(trajectory);

      rclcpp::sleep_for(std::chrono::seconds(2));

      // move_group_->execute(reverse_trajectory_msg);

      result->done = true;
     //visual_tools->deleteAllMarkers();
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
    // ! ---------------------- PourInto END ----------------------



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
// }canceled

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = std::make_shared<MoveRobotServer>(node_options);

    rclcpp::spin(move_group_node);

    rclcpp::shutdown();
    return 0;
}
