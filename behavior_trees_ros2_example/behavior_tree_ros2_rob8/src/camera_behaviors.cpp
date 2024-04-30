#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include <behaviortree_ros2/bt_service_node.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "behavior_tree_ros2_actions/action/find_aruco_tag.hpp"
#include "behavior_tree_ros2_actions/action/wait_for_user.hpp"
#include "behavior_tree_ros2_actions/srv/lookup_transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

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
    return providedBasicPorts({InputPort<int8_t>("id"),InputPort<geometry_msgs::msg::TransformStamped>("aruco_to_slot_transform"),InputPort<geometry_msgs::msg::TransformStamped>("slot_to_slot_transform"), OutputPort<geometry_msgs::msg::PoseStamped>("Transform")});
  }

  bool setGoal(Goal& goal) override{
    auto id = getInput<int8_t>("id");
    auto aruco_to_slot_transform = getInput<geometry_msgs::msg::TransformStamped>("aruco_to_slot_transform");
    auto slot_to_slot_transform = getInput<geometry_msgs::msg::TransformStamped>("slot_to_slot_transform");
    goal.id = id.value();
    goal.aruco_to_slot_transform = aruco_to_slot_transform.value();
    goal.slot_to_slot_transform = slot_to_slot_transform.value();
    return true;
  }

  void onHalt() override{
    RCLCPP_INFO( node_->get_logger(), "%s: onHalt", name().c_str() );
  }

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override{
    setOutput("Transform", wr.result->grab_pose_msg);

    // RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
    //            wr.result->done ? "true" : "false" );
    return NodeStatus::SUCCESS;//wr.result->marker_pose_msg; // ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override{
    RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
    return NodeStatus::FAILURE;
  }
};




using LookupTransform = behavior_tree_ros2_actions::srv::LookupTransform;

class LookupTransformNode: public RosServiceNode<LookupTransform>
{
  public:

  LookupTransformNode(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosServiceNode<LookupTransform>(name, conf, params)
  {}

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosServiceNode::providedBasicPorts()
  static PortsList providedPorts()
  {
    return providedBasicPorts({
        // OutputPort<std::string>("message"),
        // OutputPort<std::string>("workstation_name"),
        // OutputPort<int8_t>("aruco_id"),
        // OutputPort<bool>("empty"),
        OutputPort<bool>("result"),
        OutputPort<geometry_msgs::msg::PoseStamped>("transform"),
        // OutputPort<geometry_msgs::msg::TransformStamped>("aruco_to_slot_transform"),
        // OutputPort<geometry_msgs::msg::TransformStamped>("slot_to_slot_transform"),
        InputPort<std::string>("source"),
        InputPort<std::string>("target")});
  }

  // This is called when the TreeNode is ticked and it should
  // send the request to the service provider
  bool setRequest(Request::SharedPtr& request) override
  {
    // use input ports to set A and B
    
    getInput("target", request->target);
    getInput("source", request->source);
    // auto name_ = getInput<std::string>("name_");
    // request->name = name_.value();
    RCLCPP_INFO(node_->get_logger(), "String source: %s", request->source.c_str());
    // must return true if we are ready to send the request
    return true;
  }

  // Callback invoked when the answer is received.
  // It must return SUCCESS or FAILURE
  NodeStatus onResponseReceived(const Response::SharedPtr& response) override
  {
    RCLCPP_INFO(node_->get_logger(), "Success: %ld", response->result);
    
    setOutput("transform",response->transform);
    // setOutput("aruco_to_slot_transform",response->aruco_to_slot_transform);
    // setOutput("slot_to_slot_transform",response->slot_to_slot_transform);
    return NodeStatus::SUCCESS;
  }

  // Callback invoked when there was an error at the level
  // of the communication between client and server.
  // This will set the status of the TreeNode to either SUCCESS or FAILURE,
  // based on the return value.
  // If not overridden, it will return FAILURE by default.
  virtual NodeStatus onFailure(ServiceNodeErrorCode error) override
  {
    RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
    return NodeStatus::FAILURE;
  }
};


class WaitForUserAction: public RosActionNode<behavior_tree_ros2_actions::action::WaitForUser>
{
public:
  WaitForUserAction(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<behavior_tree_ros2_actions::action::WaitForUser>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<int8_t>("id"),InputPort<geometry_msgs::msg::TransformStamped>("aruco_to_slot_transform"),InputPort<geometry_msgs::msg::TransformStamped>("slot_to_slot_transform"), OutputPort<geometry_msgs::msg::PoseStamped>("Transform")});
  }

  bool setGoal(Goal& goal) override{
    return true;
  }

  void onHalt() override{
    RCLCPP_INFO( node_->get_logger(), "%s: onHalt", name().c_str() );
  }

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override{
    

    RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
               wr.result->done ? "true" : "false" );
    return NodeStatus::SUCCESS;
  }

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override{
    RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
    return NodeStatus::FAILURE;
  }
};