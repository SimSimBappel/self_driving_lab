#ifndef MANIPULATOR__PACKAGE_MOVE_ROBOT__SERVER_HPP_
#define MANIPULATOR__PACKAGE_MOVE_ROBOT_SERVER_HPP_


#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include "std_msgs/msg/float64_multi_array.hpp"

// gripper actions
#include "behavior_tree_ros2_actions/action/gripper.hpp"
#include "behavior_tree_ros2_actions/action/gripper_joint.hpp"

// arm actions
#include "behavior_tree_ros2_actions/action/arm_move_joints.hpp"
#include "behavior_tree_ros2_actions/action/arm_move_pose.hpp"
#include "behavior_tree_ros2_actions/action/arm_move_pose_msg.hpp"
#include "behavior_tree_ros2_actions/action/arm_move_pose_msg.hpp"
#include "behavior_tree_ros2_actions/action/arm_move_pliz_ptp_pose_msg.hpp"
#include "behavior_tree_ros2_actions/action/arm_move_pliz_lin_pose_msg.hpp"
#include "behavior_tree_ros2_actions/action/arm_move_relative_pose.hpp"
#include "behavior_tree_ros2_actions/action/arm_move_to_frame.hpp"
#include "behavior_tree_ros2_actions/action/sleep.hpp"
#include "behavior_tree_ros2_actions/action/home.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// #include "ar4_msgs/action/home.hpp"


#include "behaviortree_ros2/bt_action_node.hpp"
// INCLUDE YOUR SRVS 


class MoveRobotServer : public rclcpp::Node {
public:
    explicit MoveRobotServer(const rclcpp::NodeOptions &options);
    using Home = behavior_tree_ros2_actions::action::Home;
    
    using Gripper = behavior_tree_ros2_actions::action::Gripper;
    using GoalHandleGripper = rclcpp_action::ServerGoalHandle<Gripper>;

    using GripperJoint = behavior_tree_ros2_actions::action::GripperJoint;
    using GoalHandleGripperJoint = rclcpp_action::ServerGoalHandle<GripperJoint>;
    
    using ArmMovePose = behavior_tree_ros2_actions::action::ArmMovePose;
    using GoalHandleArmMovePose = rclcpp_action::ServerGoalHandle<ArmMovePose>;

    using ArmMovePoseMsg = behavior_tree_ros2_actions::action::ArmMovePoseMsg;
    using GoalHandleArmMovePoseMsg = rclcpp_action::ServerGoalHandle<ArmMovePoseMsg>;

    using ArmMovePlizPtpPoseMsg = behavior_tree_ros2_actions::action::ArmMovePlizPtpPoseMsg;
    using GoalHandleArmMovePlizPtpPoseMsg = rclcpp_action::ServerGoalHandle<ArmMovePlizPtpPoseMsg>;

    rclcpp_action::GoalResponse arm_move_pliz_ptp_pose_msg_handle_goal(const rclcpp_action::GoalUUID &,std::shared_ptr<const ArmMovePlizPtpPoseMsg::Goal> goal);
    rclcpp_action::CancelResponse arm_move_pliz_ptp_pose_msg_handle_cancel(const std::shared_ptr<GoalHandleArmMovePlizPtpPoseMsg> goal_handle);
    void arm_move_pliz_ptp_pose_msg_handle_accepted(const std::shared_ptr<GoalHandleArmMovePlizPtpPoseMsg> goal_handle);
    void arm_move_pliz_ptp_pose_msg_execute(const std::shared_ptr<GoalHandleArmMovePlizPtpPoseMsg> goal_handle);

    using ArmMovePlizLinPoseMsg = behavior_tree_ros2_actions::action::ArmMovePlizLinPoseMsg;
    using GoalHandleArmMovePlizLinPoseMsg = rclcpp_action::ServerGoalHandle<ArmMovePlizLinPoseMsg>;

    rclcpp_action::GoalResponse arm_move_pliz_lin_pose_msg_handle_goal(const rclcpp_action::GoalUUID &,std::shared_ptr<const ArmMovePlizLinPoseMsg::Goal> goal);
    rclcpp_action::CancelResponse arm_move_pliz_lin_pose_msg_handle_cancel(const std::shared_ptr<GoalHandleArmMovePlizLinPoseMsg> goal_handle);
    void arm_move_pliz_lin_pose_msg_handle_accepted(const std::shared_ptr<GoalHandleArmMovePlizLinPoseMsg> goal_handle);
    void arm_move_pliz_lin_pose_msg_execute(const std::shared_ptr<GoalHandleArmMovePlizLinPoseMsg> goal_handle);

    using ArmMoveJoints = behavior_tree_ros2_actions::action::ArmMoveJoints;
    using GoalHandleArmMoveJoints = rclcpp_action::ServerGoalHandle<ArmMoveJoints>;

    rclcpp_action::GoalResponse arm_move_joints_handle_goal(const rclcpp_action::GoalUUID &,std::shared_ptr<const ArmMoveJoints::Goal> goal);
    rclcpp_action::CancelResponse arm_move_joints_handle_cancel(const std::shared_ptr<GoalHandleArmMoveJoints> goal_handle);
    void arm_move_joints_handle_accepted(const std::shared_ptr<GoalHandleArmMoveJoints> goal_handle);
    void arm_move_joints_execute(const std::shared_ptr<GoalHandleArmMoveJoints> goal_handle);

    using GoalHandleHome = rclcpp_action::ServerGoalHandle<Home>;

    rclcpp_action::GoalResponse home_arm_handle_goal(const rclcpp_action::GoalUUID &,std::shared_ptr<const Home::Goal> goal);
    rclcpp_action::CancelResponse home_arm_handle_cancel(const std::shared_ptr<GoalHandleHome> goal_handle);
    void home_arm_handle_accepted(const std::shared_ptr<GoalHandleHome> goal_handle);

    rclcpp_action::GoalResponse gripper_handle_goal(const rclcpp_action::GoalUUID &,std::shared_ptr<const Gripper::Goal> goal);
    rclcpp_action::CancelResponse gripper_handle_cancel(const std::shared_ptr<GoalHandleGripper> goal_handle);
    void gripper_handle_accepted(const std::shared_ptr<GoalHandleGripper> goal_handle);
    void gripper_execute(const std::shared_ptr<GoalHandleGripper> goal_handle);

    rclcpp_action::GoalResponse arm_move_pose_handle_goal(const rclcpp_action::GoalUUID &,std::shared_ptr<const ArmMovePose::Goal> goal);
    rclcpp_action::CancelResponse arm_move_pose_handle_cancel(const std::shared_ptr<GoalHandleArmMovePose> goal_handle);
    void arm_move_pose_handle_accepted(const std::shared_ptr<GoalHandleArmMovePose> goal_handle);
    void arm_move_pose_execute(const std::shared_ptr<GoalHandleArmMovePose> goal_handle);

    rclcpp_action::GoalResponse arm_move_pose_msg_handle_goal(const rclcpp_action::GoalUUID &,std::shared_ptr<const ArmMovePoseMsg::Goal> goal);
    rclcpp_action::CancelResponse arm_move_pose_msg_handle_cancel(const std::shared_ptr<GoalHandleArmMovePoseMsg> goal_handle);
    void arm_move_pose_msg_handle_accepted(const std::shared_ptr<GoalHandleArmMovePoseMsg> goal_handle);
    void arm_move_pose_msg_execute(const std::shared_ptr<GoalHandleArmMovePoseMsg> goal_handle);

    using Sleep = behavior_tree_ros2_actions::action::Sleep;
    using GoalHandleSleep = rclcpp_action::ServerGoalHandle<Sleep>;

    rclcpp_action::GoalResponse sleep_handle_goal(const rclcpp_action::GoalUUID &,std::shared_ptr<const Sleep::Goal> goal);
    rclcpp_action::CancelResponse sleep_handle_cancel(const std::shared_ptr<GoalHandleSleep> goal_handle);
    void sleep_handle_accepted(const std::shared_ptr<GoalHandleSleep> goal_handle);
    void sleep_execute(const std::shared_ptr<GoalHandleSleep> goal_handle);


    rclcpp_action::GoalResponse gripper_joint_handle_goal(const rclcpp_action::GoalUUID &,std::shared_ptr<const GripperJoint::Goal> goal);
    rclcpp_action::CancelResponse gripper_joint_handle_cancel(const std::shared_ptr<GoalHandleGripperJoint> goal_handle);
    void gripper_joint_handle_accepted(const std::shared_ptr<GoalHandleGripperJoint> goal_handle);
    void gripper_joint_execute(const std::shared_ptr<GoalHandleGripperJoint> goal_handle);

private:
    // rclcpp::Service<your_service_msgs::srv::ExampleService>::SharedPtr service_example_server_;
    rclcpp_action::Server<Gripper>::SharedPtr action_server_gripper_;
    rclcpp_action::Server<GripperJoint>::SharedPtr action_server_gripper_joint_;
    rclcpp_action::Server<ArmMovePose>::SharedPtr action_server_arm_move_pose_;

    rclcpp_action::Server<ArmMovePoseMsg>::SharedPtr action_server_arm_move_pose_msg_;

    rclcpp_action::Server<ArmMovePlizPtpPoseMsg>::SharedPtr action_server_arm_move_pliz_ptp_pose_msg_;

    rclcpp_action::Server<ArmMovePlizLinPoseMsg>::SharedPtr action_server_arm_move_pliz_lin_pose_msg_;

    rclcpp_action::Server<ArmMoveJoints>::SharedPtr action_server_arm_move_joints_;

    rclcpp_action::Server<Home>::SharedPtr action_server_home_arm_;

    rclcpp_action::Server<Sleep>::SharedPtr action_server_sleep_;
    
    std::string node_namespace_;
    moveit::planning_interface::MoveGroupInterfacePtr move_group_;
    moveit::planning_interface::MoveGroupInterfacePtr move_gripper_group_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Executor::SharedPtr executor_;
    std::thread executor_thread_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_gripper_;

    bool MoveGripper(const std_msgs::msg::Float64MultiArray & msg);
    bool ArmMoveJ(const std_msgs::msg::Float64MultiArray & msg);
    bool Move(const geometry_msgs::msg::PoseStamped & msg);

    // void service_example_callback(const std::shared_ptr<ar4_moveit_config::srv::MoveRobot::Request> request,
    //      std::shared_ptr<ar4_moveit_config::srv::MoveRobot::Response>      response);
};

#endif //YOUR_PACKAGE_YOUR_HEADER_H