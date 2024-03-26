#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

//  msg
#include "pgsql_interfaces/msg/chemical_location.hpp"
#include "pgsql_interfaces/msg/workstation_location.hpp"
//  srv
#include "pgsql_interfaces/srv/add_chemical.hpp"
#include "pgsql_interfaces/srv/add_workstation.hpp"
#include "pgsql_interfaces/srv/upsert_chemical_location.hpp"
#include "pgsql_interfaces/srv/upsert_workstation_location.hpp"
#include "pgsql_interfaces/srv/get_all_chemical_locations.hpp"
#include "pgsql_interfaces/srv/get_all_workstation_locations.hpp"
#include "pgsql_interfaces/srv/remove_chemical.hpp"
#include "pgsql_interfaces/srv/remove_workstation.hpp"

#include <behaviortree_ros2/bt_service_node.hpp>

using AddChemical = pgsql_interfaces::srv::AddChemical;
using AddWorkstation = pgsql_interfaces::srv::AddWorkstation;
using UpsertChemicalLocation = pgsql_interfaces::srv::UpsertChemicalLocation;
using UpsertWorkstationLocation = pgsql_interfaces::srv::UpsertWorkstationLocation;
using GetAllChemicalLocations = pgsql_interfaces::srv::GetAllChemicalLocations;
using GetAllWorkstationLocations = pgsql_interfaces::srv::GetAllWorkstationLocations;
using RemoveChemical = pgsql_interfaces::srv::RemoveChemical;
using RemoveWorkstation = pgsql_interfaces::srv::RemoveWorkstation;
using namespace BT;


class AddChemicalNode: public RosServiceNode<AddChemical>
{
  public:

  AddChemicalNode(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosServiceNode<AddChemical>(name, conf, params)
  {}

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosServiceNode::providedBasicPorts()
  static PortsList providedPorts()
  {
    return providedBasicPorts({
        OutputPort<std::string>("message"),
        OutputPort<int8_t>("id_"),
        InputPort<std::string>("name_"),
        InputPort<int8_t>("safety_level"),
        InputPort<std::string>("formula")});
  }

  // This is called when the TreeNode is ticked and it should
  // send the request to the service provider
  bool setRequest(Request::SharedPtr& request) override
  {
    // use input ports to set A and B
    getInput("name_", request->name);
    getInput("formula", request->formula);
    getInput("safety_level", request->safety_level);
    // must return true if we are ready to send the request
    return true;
  }

  // Callback invoked when the answer is received.
  // It must return SUCCESS or FAILURE
  NodeStatus onResponseReceived(const Response::SharedPtr& response) override
  {
    RCLCPP_INFO(node_->get_logger(), "Sum: %ld", response->success);
    setOutput("id_",response->id);
    setOutput("message",response->message);
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




class AddWorkstationNode: public RosServiceNode<AddWorkstation>
{
  public:

  AddWorkstationNode(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosServiceNode<AddWorkstation>(name, conf, params)
  {}

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosServiceNode::providedBasicPorts()
  static PortsList providedPorts()
  {
    return providedBasicPorts({
        OutputPort<std::string>("message"),
        OutputPort<int8_t>("id_"),
        InputPort<std::string>("name_"),
        
        InputPort<std::string>("type")});
  }

  // This is called when the TreeNode is ticked and it should
  // send the request to the service provider
  bool setRequest(Request::SharedPtr& request) override
  {
    // use input ports to set A and B
    getInput("name_", request->name);
    getInput("type", request->type);
    
    // must return true if we are ready to send the request
    return true;
  }

  // Callback invoked when the answer is received.
  // It must return SUCCESS or FAILURE
  NodeStatus onResponseReceived(const Response::SharedPtr& response) override
  {
    RCLCPP_INFO(node_->get_logger(), "Sum: %ld", response->success);
    setOutput("id_",response->id);
    setOutput("message",response->message);
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



class UpsertChemicalLocationNode: public RosServiceNode<UpsertChemicalLocation>
{
  public:

  UpsertChemicalLocationNode(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosServiceNode<UpsertChemicalLocation>(name, conf, params)
  {}

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosServiceNode::providedBasicPorts()
  static PortsList providedPorts()
  {
    return providedBasicPorts({
        OutputPort<std::string>("message"),
        InputPort<int8_t>("id_"),
        InputPort<int8_t>("location_id"),
        InputPort<std::string>("name_"),
        InputPort<std::string>("formula"),
        InputPort<geometry_msgs::msg::Pose>("location_base"),
        InputPort<geometry_msgs::msg::Pose>("location_hand")});

  }

  // This is called when the TreeNode is ticked and it should
  // send the request to the service provider
  bool setRequest(Request::SharedPtr& request) override
  {
    // use input ports to set A and B
    getInput("name_", request->name);
    getInput("formula", request->formula);
    getInput("id_", request->id);
    getInput("location_id", request->location_id);
    getInput("location_base", request->location_base);
    getInput("location_hand", request->location_hand);
    // must return true if we are ready to send the request
    return true;
  }

  // Callback invoked when the answer is received.
  // It must return SUCCESS or FAILURE
  NodeStatus onResponseReceived(const Response::SharedPtr& response) override
  {
    RCLCPP_INFO(node_->get_logger(), "Sum: %ld", response->success);
    setOutput("message",response->message);
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



class UpsertWorkstationLocationNode: public RosServiceNode<UpsertWorkstationLocation>
{
  public:

  UpsertWorkstationLocationNode(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosServiceNode<UpsertWorkstationLocation>(name, conf, params)
  {}

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosServiceNode::providedBasicPorts()
  static PortsList providedPorts()
  {
    return providedBasicPorts({
        OutputPort<std::string>("message"),
        InputPort<int8_t>("id_"),
        InputPort<int8_t>("location_id"),
        InputPort<std::string>("name_"),
        InputPort<geometry_msgs::msg::Pose>("location_base"),
        InputPort<geometry_msgs::msg::Pose>("location_hand")});

  }

  // This is called when the TreeNode is ticked and it should
  // send the request to the service provider
  bool setRequest(Request::SharedPtr& request) override
  {
    // use input ports to set A and B
    getInput("name_", request->name);
    getInput("id_", request->id);
    getInput("location_id", request->location_id);
    getInput("location_base", request->location_base);
    getInput("location_hand", request->location_hand);
    // must return true if we are ready to send the request
    return true;
  }

  // Callback invoked when the answer is received.
  // It must return SUCCESS or FAILURE
  NodeStatus onResponseReceived(const Response::SharedPtr& response) override
  {
    RCLCPP_INFO(node_->get_logger(), "Sum: %ld", response->success);
    setOutput("message",response->message);
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




class GetAllChemicalLocationsNode: public RosServiceNode<GetAllChemicalLocations>
{
  public:

  GetAllChemicalLocationsNode(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosServiceNode<GetAllChemicalLocations>(name, conf, params)
  {}

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosServiceNode::providedBasicPorts()
  static PortsList providedPorts()
  {
    return providedBasicPorts({
        OutputPort<std::string>("message"),
        OutputPort<std::vector<pgsql_interfaces::msg::ChemicalLocation>>("locations"),
        InputPort<int8_t>("id_"),
        InputPort<std::string>("formula"),
        InputPort<std::string>("name_"),});

  }

  // This is called when the TreeNode is ticked and it should
  // send the request to the service provider
  bool setRequest(Request::SharedPtr& request) override
  {
    // use input ports to set A and B
    getInput("name_", request->name);
    getInput("id_", request->id);
    getInput("formula", request->formula);
    
    // must return true if we are ready to send the request
    return true;
  }

  // Callback invoked when the answer is received.
  // It must return SUCCESS or FAILURE
  NodeStatus onResponseReceived(const Response::SharedPtr& response) override
  {
    RCLCPP_INFO(node_->get_logger(), "Sum: %ld", response->success);
    setOutput("message",response->message);
    setOutput("locations",response->locations);
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




class GetAllWorkstationLocationsNode: public RosServiceNode<GetAllWorkstationLocations>
{
  public:

  GetAllWorkstationLocationsNode(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosServiceNode<GetAllWorkstationLocations>(name, conf, params)
  {}

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosServiceNode::providedBasicPorts()
  static PortsList providedPorts()
  {
    return providedBasicPorts({
        OutputPort<std::string>("message"),
        OutputPort<std::vector<pgsql_interfaces::msg::WorkstationLocation>>("locations"),
        InputPort<int8_t>("id_"),
        InputPort<std::string>("formula"),
        InputPort<std::string>("name_"),});

  }

  // This is called when the TreeNode is ticked and it should
  // send the request to the service provider
  bool setRequest(Request::SharedPtr& request) override
  {
    // use input ports to set A and B
    getInput("name_", request->name);
    getInput("id_", request->id);
    getInput("formula", request->formula);
    
    // must return true if we are ready to send the request
    return true;
  }

  // Callback invoked when the answer is received.
  // It must return SUCCESS or FAILURE
  NodeStatus onResponseReceived(const Response::SharedPtr& response) override
  {
    RCLCPP_INFO(node_->get_logger(), "Sum: %ld", response->success);
    setOutput("message",response->message);
    setOutput("locations",response->locations);
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


class RemoveChemicalNode: public RosServiceNode<RemoveChemical>
{
  public:

  RemoveChemicalNode(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosServiceNode<RemoveChemical>(name, conf, params)
  {}

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosServiceNode::providedBasicPorts()
  static PortsList providedPorts()
  {
    return providedBasicPorts({
        OutputPort<std::string>("message"),
        InputPort<int32_t>("chemical_id"),
        InputPort<int32_t>("location_id"),});

  }

  // This is called when the TreeNode is ticked and it should
  // send the request to the service provider
  bool setRequest(Request::SharedPtr& request) override
  {
    // use input ports to set A and B
    getInput("chemical_id", request->chemical_id);
    getInput("location_id", request->location_id);
    
    // must return true if we are ready to send the request
    return true;
  }

  // Callback invoked when the answer is received.
  // It must return SUCCESS or FAILURE
  NodeStatus onResponseReceived(const Response::SharedPtr& response) override
  {
    RCLCPP_INFO(node_->get_logger(), "Sum: %ld", response->success);
    setOutput("message",response->message);
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




class RemoveWorkstationNode: public RosServiceNode<RemoveWorkstation>
{
  public:

  RemoveWorkstationNode(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosServiceNode<RemoveWorkstation>(name, conf, params)
  {}

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosServiceNode::providedBasicPorts()
  static PortsList providedPorts()
  {
    return providedBasicPorts({
        OutputPort<std::string>("message"),
        InputPort<int32_t>("workstation_id"),
        InputPort<int32_t>("location_id"),});

  }

  // This is called when the TreeNode is ticked and it should
  // send the request to the service provider
  bool setRequest(Request::SharedPtr& request) override
  {
    // use input ports to set A and B
    getInput("workstation_id", request->workstation_id);
    getInput("location_id", request->location_id);
    
    // must return true if we are ready to send the request
    return true;
  }

  // Callback invoked when the answer is received.
  // It must return SUCCESS or FAILURE
  NodeStatus onResponseReceived(const Response::SharedPtr& response) override
  {
    RCLCPP_INFO(node_->get_logger(), "Sum: %ld", response->success);
    setOutput("message",response->message);
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