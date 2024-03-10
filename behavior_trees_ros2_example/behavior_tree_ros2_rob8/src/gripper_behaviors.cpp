// #include "home_action.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"


#include "behavior_tree_ros2_actions/action/gripper.hpp"
#include "behavior_tree_ros2_actions/action/gripper_joint.hpp"



using namespace BT;

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
    return providedBasicPorts({InputPort<unsigned>("open")});
  }

  bool setGoal(Goal& goal) override{
    auto open = getInput<bool>("open");
    goal.open = open.value();
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
    return providedBasicPorts({InputPort<double>("joint_1"), InputPort<double>("joint_2")});
  }

  bool setGoal(Goal& goal) override{
    auto joint1 = getInput<double>("joint_1");
    auto joint2 = getInput<double>("joint_2");
    goal.joint_1 = joint1.value();
    goal.joint_2 = joint2.value();
    
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