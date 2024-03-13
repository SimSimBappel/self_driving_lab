#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "behavior_tree_ros2_actions/action/find_aruco_tag.hpp"

using namespace BT;

class FindArucoTagAction: public RosActionNode<behavior_tree_ros2_actions::action::FindArucoTag>
{
public:
  FindArucoTagAction(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<behavior_tree_ros2_actions::action::FindArucoTag>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<int>("id"), OutputPort<geometry_msgs::msg::TransformStamped>("Transform")});
  }

  bool setGoal(Goal& goal) override{
    auto id = getInput<int>("tag_id");
    goal.id = id.value();
    return true;
  }

  void onHalt() override{
    RCLCPP_INFO( node_->get_logger(), "%s: onHalt", name().c_str() );
  }

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override{
    setOutput("Transform", wr.result->marker_pose_msg);

    // RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
    //            wr.result->done ? "true" : "false" );
    return NodeStatus::SUCCESS;//wr.result->marker_pose_msg; // ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override{
    RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
    return NodeStatus::FAILURE;
  }
};
