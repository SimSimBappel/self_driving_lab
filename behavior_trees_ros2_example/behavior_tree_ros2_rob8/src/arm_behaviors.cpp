// #include "home_action.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "behavior_tree_ros2_actions/action/arm_move_joints.hpp"
#include "behavior_tree_ros2_actions/action/arm_move_pose.hpp"
#include "behavior_tree_ros2_actions/action/arm_move_relative_pose.hpp"
#include "behavior_tree_ros2_actions/action/arm_move_to_frame.hpp"


using namespace BT;

class ArmMoveJointsAction: public RosActionNode<behavior_tree_ros2_actions::action::ArmMoveJoints>
{
public:
  ArmMoveJointsAction(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<behavior_tree_ros2_actions::action::ArmMoveJoints>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<std::vector<double>>("joints")});
  }

  bool setGoal(Goal& goal) override{
    auto frame = getInput<std::vector<double>>("joints");
    goal.joints = frame.value();
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

class ArmMoveToFrameAction: public RosActionNode<behavior_tree_ros2_actions::action::ArmMoveToFrame>
{
public:
  ArmMoveToFrameAction(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<behavior_tree_ros2_actions::action::ArmMoveToFrame>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<std::string>("frame")});
  }

  bool setGoal(Goal& goal) override{
    auto frame = getInput<std::string>("frame");
    goal.frame = frame.value();
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

class ArmMovePoseAction: public RosActionNode<behavior_tree_ros2_actions::action::ArmMovePose>
{
public:
  ArmMovePoseAction(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<behavior_tree_ros2_actions::action::ArmMovePose>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<std::string>("pose")});
  }

  bool setGoal(Goal& goal) override{
    // auto pose = getInput<std::vector<double>>("pose");
    
    
    // goal.pose = pose.value();
    //  auto pose = getInput<std::string>("pose");
    //  goal.pose = pose.value();
     auto pose = getInput<std::string>("pose");
    goal.pose = pose.value();
    
    
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