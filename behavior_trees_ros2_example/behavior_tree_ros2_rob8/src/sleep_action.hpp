#include "behaviortree_ros2/bt_action_node.hpp"
#include "behavior_tree_ros2_actions/action/sleep.hpp"

using namespace BT;

class SleepAction: public RosActionNode<behavior_tree_ros2_actions::action::Sleep>
{
public:
  SleepAction(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<behavior_tree_ros2_actions::action::Sleep>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<unsigned>("msec")});
  }

  bool setGoal(Goal& goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};
