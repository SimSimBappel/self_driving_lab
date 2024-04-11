// #include "home_action.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"
#include <behaviortree_ros2/bt_service_node.hpp>
#include "behaviortree_ros2/plugins.hpp"
#include "behavior_tree_ros2_actions/action/arm_move_joints.hpp"
#include "behavior_tree_ros2_actions/action/arm_move_pose.hpp"
#include "behavior_tree_ros2_actions/action/arm_move_pose_msg.hpp"
#include "behavior_tree_ros2_actions/action/arm_move_pose_msg_tcp.hpp"
#include "behavior_tree_ros2_actions/action/arm_move_pliz_ptp_pose_msg.hpp"
#include "behavior_tree_ros2_actions/action/arm_move_pliz_lin_pose_msg.hpp"
#include "behavior_tree_ros2_actions/action/arm_move_relative_pose.hpp"
#include "behavior_tree_ros2_actions/action/arm_move_to_frame.hpp"
#include "behavior_tree_ros2_actions/action/home.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "tf2/LinearMath/Quaternion.h"
// #include <tf2/LinearMath/Quaternion.h>


#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_srvs/srv/empty.hpp>

using namespace BT;

using Empty = std_srvs::srv::Empty;
class ClearOctomapNode: public RosServiceNode<Empty>
{
  public:

  ClearOctomapNode(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosServiceNode<Empty>(name, conf, params)
  {}

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosServiceNode::providedBasicPorts()
  static PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  // This is called when the TreeNode is ticked and it should
  // send the request to the service provider
  bool setRequest(Request::SharedPtr& request) override
  {
    // use input ports to set A and B
    
    // getInput("name_", request->name);
    // // auto name_ = getInput<std::string>("name_");
    // // request->name = name_.value();
    // RCLCPP_INFO(node_->get_logger(), "String chemical: %s", request->name.c_str());
    // must return true if we are ready to send the request
    return true;
  }

  // Callback invoked when the answer is received.
  // It must return SUCCESS or FAILURE
  NodeStatus onResponseReceived(const Response::SharedPtr& response) override
  {
    // RCLCPP_INFO(node_->get_logger(), "Success: %ld", response->success);
    // setOutput("aruco_id",response->aruco_id);
    // setOutput("message",response->message);
    // setOutput("success",response->success);
    // setOutput("empty",response->empty);
    // setOutput("workstation_name",response->workstation_name);
    // setOutput("lookout_pose",response->lookout_pose);
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


class ArmArrayToPoseAction : public BT::SyncActionNode
{
public:
  ArmArrayToPoseAction(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
  {}

  // This Action simply write a value in the port "text"
  BT::NodeStatus tick() override
  { 
    auto array = getInput<std::vector<double>>("array");
    std::vector<double> array_ = array.value();
    geometry_msgs::msg::PoseStamped pose_goal;
    pose_goal.pose.position.x = array_[0];
    pose_goal.pose.position.y = array_[1];
    pose_goal.pose.position.z = array_[2];

    tf2::Quaternion q_new;
    q_new.setRPY(array_[3], array_[4], array_[5]);
  

    q_new.normalize();
    pose_goal.header.frame_id = "panda_link0";
    pose_goal.pose.orientation.x = q_new.x();
    pose_goal.pose.orientation.y = q_new.y();
    pose_goal.pose.orientation.z = q_new.z();
    pose_goal.pose.orientation.w = q_new.w();
    

    setOutput("pose", pose_goal);
    
    return BT::NodeStatus::SUCCESS;
  }

  // A node having ports MUST implement this STATIC method
  static BT::PortsList providedPorts()
  {
    return {BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose"),InputPort<std::vector<double>>("array")};
  }
};


class ArmPoseOffsetCalculation : public BT::SyncActionNode
{
public:
  ArmPoseOffsetCalculation(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
  {}

  // This Action simply write a value in the port "text"
  BT::NodeStatus tick() override
  { 
    auto offset = getInput<std::vector<double>>("offset");
    auto pose = getInput<std::vector<double>>("pose");
    std::vector<double> pose_ = pose.value();
    std::vector<double> offset_ = offset.value();
    for(int i = 0; i<pose_.size();i++)
    {
      pose_[i] = pose_[i]-offset_[i];
    }

    setOutput("pose_w_offset", pose_);
    
    return BT::NodeStatus::SUCCESS;
  }

  // A node having ports MUST implement this STATIC method
  static BT::PortsList providedPorts()
  {
    return {BT::OutputPort<std::vector<double>>("pose_w_offset"),InputPort<std::vector<double>>("pose"),InputPort<std::vector<double>>("offset")};
  }
};

class ArmPoseMsgOffsetCalculation : public BT::SyncActionNode
{
public:
  ArmPoseMsgOffsetCalculation(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
  {}

  // This Action simply write a value in the port "text"
  BT::NodeStatus tick() override
  { 
    auto offset = getInput<std::vector<double>>("offset");
    auto pose = getInput<geometry_msgs::msg::PoseStamped>("pose");
    geometry_msgs::msg::PoseStamped pose_goal = pose.value();
    std::vector<double> offset_ = offset.value();
    
    pose_goal.pose.position.x -= offset_[0];
    pose_goal.pose.position.y -= offset_[1];
    pose_goal.pose.position.z -= offset_[2];

    tf2::Quaternion q_rot,q_new,quat_from_msg;
    tf2::fromMsg(pose_goal.pose.orientation, quat_from_msg);
    q_rot.setRPY(offset_[3], offset_[4], offset_[5]);
  

    q_rot.normalize();

    q_new = q_rot * quat_from_msg;
    q_new.normalize();



    tf2::convert(pose_goal.pose.orientation, q_new);
    
    setOutput("pose_w_offset", pose_goal);
    
    return BT::NodeStatus::SUCCESS;
  }

  // A node having ports MUST implement this STATIC method
  static BT::PortsList providedPorts()
  {
    return {BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose_w_offset"),InputPort<geometry_msgs::msg::PoseStamped>("pose"),InputPort<std::vector<double>>("offset")};
  }
};



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
    return providedBasicPorts({InputPort<std::vector<double>>("joints"),InputPort<double>("speed"),InputPort<double>("accel")});
  }

  bool setGoal(Goal& goal) override{
    auto frame = getInput<std::vector<double>>("joints");
    goal.joints = frame.value();
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

class HomeArmAction: public RosActionNode<behavior_tree_ros2_actions::action::Home>
{
public:
  HomeArmAction(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<behavior_tree_ros2_actions::action::Home>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  bool setGoal(Goal& goal) override{
    // auto frame = getInput<std::vector<double>>("joints");
    // goal.joints = frame.value();
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
    // return providedBasicPorts({InputPort<std::string>("pose")});
    return providedBasicPorts({InputPort<std::vector<double>>("pose"),InputPort<double>("speed"),InputPort<double>("accel"),InputPort<bool>("keep_orientation")});
  }

  bool setGoal(Goal& goal) override{
    auto pos = getInput<std::vector<double>>("pose");
    
    
    // goal.pose = pose.value();
    //  auto pose = getInput<std::string>("pose");
    //  goal.pose = pose.value();
    //  auto pose = getInput<std::string>("pose");
    auto speed = getInput<double>("speed");
    auto accel = getInput<double>("accel");
    goal.speed = speed.value();
    goal.accel = accel.value();
    goal.pose = pos.value();
    auto ko =getInput<bool>("keep_orientation");
    goal.keep_orientation = ko.value();
    
    
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

class ArmMovePoseMsgAction: public RosActionNode<behavior_tree_ros2_actions::action::ArmMovePoseMsg>
{
public:
  ArmMovePoseMsgAction(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<behavior_tree_ros2_actions::action::ArmMovePoseMsg>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    // return providedBasicPorts({InputPort<std::string>("pose")});
    return providedBasicPorts({InputPort<geometry_msgs::msg::PoseStamped>("pose"),InputPort<double>("speed"),InputPort<double>("accel"),InputPort<bool>("keep_orientation")});
  }

  bool setGoal(Goal& goal) override{
    auto pose = getInput<geometry_msgs::msg::PoseStamped>("pose");
    
    
    // goal.pose = pose.value();
    //  auto pose = getInput<std::string>("pose");
    //  goal.pose = pose.value();
    //  auto pose = getInput<std::string>("pose");
    goal.pose = pose.value();
    auto speed = getInput<double>("speed");
    auto accel = getInput<double>("accel");
    goal.speed = speed.value();
    goal.accel = accel.value();
    auto ko = getInput<bool>("keep_orientation");
    goal.keep_orientation = ko.value();
    
    
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

class ArmMovePoseMsgTcpAction: public RosActionNode<behavior_tree_ros2_actions::action::ArmMovePoseMsgTcp>
{
public:
  ArmMovePoseMsgTcpAction(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<behavior_tree_ros2_actions::action::ArmMovePoseMsgTcp>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    // return providedBasicPorts({InputPort<std::string>("pose")});
    return providedBasicPorts({InputPort<geometry_msgs::msg::PoseStamped>("pose"),InputPort<double>("speed"),InputPort<double>("accel"),InputPort<std::string>("tcp_frame")});
  }

  bool setGoal(Goal& goal) override{
    auto pose = getInput<geometry_msgs::msg::PoseStamped>("pose");
    
    
    // goal.pose = pose.value();
    //  auto pose = getInput<std::string>("pose");
    //  goal.pose = pose.value();
    //  auto pose = getInput<std::string>("pose");
    goal.pose = pose.value();
    auto speed = getInput<double>("speed");
    auto accel = getInput<double>("accel");
    auto tcp_frame = getInput<std::string>("tcp_frame");
    goal.speed = speed.value();
    goal.accel = accel.value();
    goal.tcp_frame = tcp_frame.value();
    
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



class ArmMovePlizPtpPoseMsgAction: public RosActionNode<behavior_tree_ros2_actions::action::ArmMovePlizPtpPoseMsg>
{
public:
  ArmMovePlizPtpPoseMsgAction(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<behavior_tree_ros2_actions::action::ArmMovePlizPtpPoseMsg>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    // return providedBasicPorts({InputPort<std::string>("pose")});
    return providedBasicPorts({InputPort<geometry_msgs::msg::PoseStamped>("pose"),InputPort<double>("speed"),InputPort<double>("accel")});
  }

  bool setGoal(Goal& goal) override{
    auto pose = getInput<geometry_msgs::msg::PoseStamped>("pose");
    auto speed = getInput<double>("speed");
    auto accel = getInput<double>("accel");
    
    // goal.pose = pose.value();
    //  auto pose = getInput<std::string>("pose");
    //  goal.pose = pose.value();
    //  auto pose = getInput<std::string>("pose");
    goal.pose = pose.value();
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

class ArmMovePlizLinPoseMsgAction: public RosActionNode<behavior_tree_ros2_actions::action::ArmMovePlizLinPoseMsg>
{
public:
  ArmMovePlizLinPoseMsgAction(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<behavior_tree_ros2_actions::action::ArmMovePlizLinPoseMsg>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    // return providedBasicPorts({InputPort<std::string>("pose")});
    return providedBasicPorts({InputPort<geometry_msgs::msg::PoseStamped>("pose"),InputPort<double>("speed"),InputPort<double>("accel")});
  }

  bool setGoal(Goal& goal) override{
    auto pose = getInput<geometry_msgs::msg::PoseStamped>("pose");
    auto speed = getInput<double>("speed");
    auto accel = getInput<double>("accel");
    
    // goal.pose = pose.value();
    //  auto pose = getInput<std::string>("pose");
    //  goal.pose = pose.value();
    //  auto pose = getInput<std::string>("pose");
    goal.pose = pose.value();
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