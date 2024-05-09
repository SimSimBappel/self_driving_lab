#include "behaviortree_ros2/bt_action_node.hpp"
#include <behaviortree_ros2/bt_service_node.hpp>
#include "behaviortree_ros2/plugins.hpp"

#include "behavior_tree_ros2_actions/action/mir_mission.hpp"
using namespace BT;
// using MirMission = behavior_tree_ros2_actions::action::MirMission;

class MirMissionAction: public RosActionNode<behavior_tree_ros2_actions::action::MirMission>{
public:
  MirMissionAction(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<behavior_tree_ros2_actions::action::MirMission>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<std::string>("mission_id"),OutputPort<bool>("result")});
  }

  bool setGoal(Goal& goal) override{
    auto mission_id = getInput<std::string>("mission_id");
    goal.mission_id = mission_id.value();
    return true;
  }

  void onHalt() override{
    RCLCPP_INFO( node_->get_logger(), "%s: onHalt", name().c_str() );
  }

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override{
    RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
               wr.result->done ? "true" : "false" );
    setOutput("result", wr.result->done);
    return wr.result->done ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override{
    RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
    return NodeStatus::FAILURE;
  }
};



class MirCheckPostion: public RosActionNode<behavior_tree_ros2_actions::action::MirMission>{
public:
  MirCheckPostion(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<behavior_tree_ros2_actions::action::MirMission>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<std::string>("mission_id"),OutputPort<bool>("result")});
  }

  bool setGoal(Goal& goal) override{
    auto mission_id = getInput<std::string>("mission_id");
    goal.mission_id = mission_id.value();
    return true;
  }

  void onHalt() override{
    RCLCPP_INFO( node_->get_logger(), "%s: onHalt", name().c_str() );
  }

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override{
    RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
               wr.result->done ? "true" : "false" );
    setOutput("result", wr.result->done);
    return wr.result->done ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override{
    RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
    return NodeStatus::FAILURE;
  }
};