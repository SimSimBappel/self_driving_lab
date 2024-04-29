// #include "home_action.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"


#include "behavior_tree_ros2_actions/action/gripper.hpp"
#include "behavior_tree_ros2_actions/action/gripper_joint.hpp"

#include "franka_msgs/action/grasp.hpp"
#include "franka_msgs/action/homing.hpp"
#include "franka_msgs/action/move.hpp"
#include "franka_msgs/msg/grasp_epsilon.h"


using namespace BT;

class FrankaGraspGripperAction: public RosActionNode<franka_msgs::action::Grasp>
{
public:
  FrankaGraspGripperAction(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<franka_msgs::action::Grasp>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<double>("width"),InputPort<double>("speed"),InputPort<double>("force"),InputPort<double>("epsilon")});
  }

  bool setGoal(Goal& goal) override{

    franka_msgs::msg::GraspEpsilon eps;

    // auto open = getInput<bool>("open");
    // goal.open = open.value();
    auto force = getInput<double>("force");
    auto speed = getInput<double>("speed");
    auto width = getInput<double>("width");
    auto epsilon = getInput<double>("epsilon");
    goal.speed = speed.value();
    goal.width = width.value();
    goal.force = force.value();
    goal.epsilon.inner = epsilon.value();
    goal.epsilon.outer = epsilon.value();
    RCLCPP_INFO( node_-> get_logger(), name().c_str(), goal);
    return true;
  }

  void onHalt() override{
    RCLCPP_INFO( node_->get_logger(), "%s: onHalt", name().c_str() );
  }

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override{
    RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
               wr.result->success ? "true" : "false" );

    return wr.result->success ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override{
    RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
    return NodeStatus::FAILURE;
  }
};

class FrankaMoveGripperAction: public RosActionNode<franka_msgs::action::Move>
{
public:
  FrankaMoveGripperAction(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<franka_msgs::action::Move>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<double>("width"),InputPort<double>("speed")});
  }

  bool setGoal(Goal& goal) override{


    // auto open = getInput<bool>("open");
    // goal.open = open.value();
    auto speed = getInput<double>("speed");
    auto width = getInput<double>("width");
    goal.speed = speed.value();
    goal.width = width.value();
    return true;
  }

  void onHalt() override{
    RCLCPP_INFO( node_->get_logger(), "%s: onHalt", name().c_str() );
  }

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override{
    RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
               wr.result->success ? "true" : "false" );

    return wr.result->success ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override{
    RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
    return NodeStatus::FAILURE;
  }
};


class FrankaHomeGripperAction: public RosActionNode<franka_msgs::action::Homing>
{
public:
  FrankaHomeGripperAction(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<franka_msgs::action::Homing>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  bool setGoal(Goal& goal) override{
    // auto open = getInput<bool>("open");
    // goal.open = open.value();
    // auto speed = getInput<double>("speed");
    // auto accel = getInput<double>("accel");
    // goal.speed = speed.value();
    // goal.accel = accel.value();
    return true;
  }

  void onHalt() override{
    RCLCPP_INFO( node_->get_logger(), "%s: onHalt", name().c_str() );
  }

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override{
    RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
               wr.result->success ? "true" : "false" );

    return wr.result->success ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override{
    RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
    return NodeStatus::FAILURE;
  }
};

class GripperAction: public RosActionNode<behavior_tree_ros2_actions::action::Gripper>
{
public:
  GripperAction(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<behavior_tree_ros2_actions::action::Gripper>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<unsigned>("open"),InputPort<double>("speed"),InputPort<double>("accel")});
  }

  bool setGoal(Goal& goal) override{
    auto open = getInput<bool>("open");
    goal.open = open.value();
    auto speed = getInput<double>("speed");
    auto accel = getInput<double>("accel");
    goal.speed = speed.value();
    goal.accel = accel.value();
    return true;
  }

  void onHalt() override{
    RCLCPP_INFO( node_->get_logger(), "%s: onHalt", name().c_str() );
  }

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override{
    RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
               wr.result->done ? "true" : "false" );

    return wr.result->done ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override{
    RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
    return NodeStatus::FAILURE;
  }
};

using namespace BT;

class GripperJointAction: public RosActionNode<behavior_tree_ros2_actions::action::GripperJoint>
{
public:
  GripperJointAction(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<behavior_tree_ros2_actions::action::GripperJoint>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<double>("joint_1"), InputPort<double>("joint_2"),InputPort<double>("speed"),InputPort<double>("accel")});
  }

  bool setGoal(Goal& goal) override{
    auto joint1 = getInput<double>("joint_1");
    auto joint2 = getInput<double>("joint_2");
    goal.joint_1 = joint1.value();
    goal.joint_2 = joint2.value();
    auto speed = getInput<double>("speed");
    auto accel = getInput<double>("accel");
    goal.speed = speed.value();
    goal.accel = accel.value();
    
    return true;
  }

  void onHalt() override{
    RCLCPP_INFO( node_->get_logger(), "%s: onHalt", name().c_str() );
  }

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override{
    RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
               wr.result->done ? "true" : "false" );

    return wr.result->done ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override{
    RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
    return NodeStatus::FAILURE;
  }
};