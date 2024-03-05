#include "behaviortree_ros2/bt_action_node.hpp"
#include "behavior_tree_ros2_actions/action/home.hpp"


using namespace BT;

class HomeAction: public RosActionNode<behavior_tree_ros2_actions::action::Home>
{
public:
  HomeAction(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<behavior_tree_ros2_actions::action::Home>(name, conf, params)
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
