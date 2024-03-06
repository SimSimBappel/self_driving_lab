#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>



#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
// #include <unistd.h>


/// https://moveit.picknik.ai/main/doc/examples/planning_scene_ros_api/planning_scene_ros_api_tutorial.html


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_demo", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "panda_1";
  static const std::string GRIPPER_GROUP = "panda_1_gripper";
  // Create a ROS logger
  auto const LOGGER = rclcpp::get_logger("move_demo");

  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface move_group_gripper(move_group_node, GRIPPER_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  const moveit::core::JointModelGroup* joint_model_group_gripper =
      move_group.getCurrentState()->getJointModelGroup(GRIPPER_GROUP);

  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  std::vector<double> joints = {0.0228318531, 0.0228318531};
  bool success;
  std::vector<std::string> joint_names = {"panda1_finger_joint1", "panda1_finger_joint2"};
  move_group_gripper.setJointValueTarget(joint_names,joints);
  success = static_cast<bool>(move_group_gripper.plan(my_plan));
  RCLCPP_INFO(LOGGER, " (gripper On) %s", success ? "" : "FAILED");
  if(success == true){
      //move_group.move();
      move_group_gripper.execute(my_plan);

  }

  sleep(1);

  move_group.setNumPlanningAttempts(10);
    move_group.setPoseReferenceFrame("panda1_link0");
    
    move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
  // Next step goes here

  //move_group.setGoalOrientationTolerance(0.2);
  

  move_group.setPlanningTime(15.0);
  move_group.setPlanningPipelineId("ompl");
  // move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
  // move_group.setPlannerId("PTP");
  move_group.setMaxAccelerationScalingFactor(0.6);
  move_group.setMaxVelocityScalingFactor(0.6);
  
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(-3.14,0.0,0.0);
    quat_tf.normalize();
  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.x = quat_tf.x();
  target_pose1.orientation.y = quat_tf.y();
  target_pose1.orientation.z = quat_tf.z();
  target_pose1.orientation.w = quat_tf.w();


  // target_pose1.position.x = 1.1;
  // target_pose1.position.y = 1.0;
  // target_pose1.position.z = 1.0;
  target_pose1.position.x = 0.623;
  target_pose1.position.y = -0.1;
  target_pose1.position.z = 0.281;
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(target_pose1);

 

  success = static_cast<bool>(move_group.plan(my_plan));
              RCLCPP_INFO(LOGGER, " (movement) %s", success ? "" : "FAILED");
              if(success == true){
                  //move_group.move();
                  move_group.execute(my_plan);
                  
                  

                  
              }
  
  sleep(1);

  //bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  joints = {0.000, 0.000};
  
  joint_names = {"panda1_finger_joint1", "panda1_finger_joint2"};
  move_group_gripper.setJointValueTarget(joint_names,joints);
  success = static_cast<bool>(move_group_gripper.plan(my_plan));
  RCLCPP_INFO(LOGGER, " (gripper On) %s", success ? "" : "FAILED");
  if(success == true){
      //move_group.move();
      move_group_gripper.execute(my_plan);

  }

  sleep(3);


  // joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  
  // joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  // move_group.setJointValueTarget(joint_names,joints);
  // success = static_cast<bool>(move_group.plan(my_plan));
  // RCLCPP_INFO(LOGGER, " (gripper On) %s", success ? "" : "FAILED");
  // if(success == true){
  //     //move_group.move();
  //     move_group.execute(my_plan);

  // }

  //  sleep(3);

  // quat_tf.setRPY(-1.589,0.0,3.142);
  //   quat_tf.normalize();
  target_pose1.orientation.x = quat_tf.x();
  target_pose1.orientation.y = quat_tf.y();
  target_pose1.orientation.z = quat_tf.z();
  target_pose1.orientation.w = quat_tf.w();


  target_pose1.position.x = 0.623;
  target_pose1.position.y = -0.1;
  target_pose1.position.z = 0.281;
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(target_pose1);

 

  success = static_cast<bool>(move_group.plan(my_plan));
              RCLCPP_INFO(LOGGER, " (movement) %s", success ? "" : "FAILED");
              if(success == true){
                  //move_group.move();
                  move_group.execute(my_plan);
                  

                  
              }
  
  // move_group.setPlannerId("LIN");
  move_group.setMaxAccelerationScalingFactor(0.8);
  move_group.setMaxVelocityScalingFactor(0.8);
  
  
  
  
  while(true){


   sleep(1);
  quat_tf.setRPY(-3.142,0.0,0.0);
    quat_tf.normalize();
  target_pose1.orientation.x = quat_tf.x();
  target_pose1.orientation.y = quat_tf.y();
  target_pose1.orientation.z = quat_tf.z();
  target_pose1.orientation.w = quat_tf.w();


  target_pose1.position.x = 0.623;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.281;
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(target_pose1);

 

  success = static_cast<bool>(move_group.plan(my_plan));
              RCLCPP_INFO(LOGGER, " (movement) %s", success ? "" : "FAILED");
              if(success == true){
                  //move_group.move();
                  move_group.execute(my_plan);
                  

                  
              }
  
  sleep(1);

  joints = {0.000, 0.000};
  
  joint_names = {"panda1_finger_joint1", "panda1_finger_joint2"};
  move_group_gripper.setJointValueTarget(joint_names,joints);
  success = static_cast<bool>(move_group_gripper.plan(my_plan));
  RCLCPP_INFO(LOGGER, " (gripper On) %s", success ? "" : "FAILED");
  if(success == true){
      //move_group.move();
      move_group_gripper.execute(my_plan);

  }

  sleep(1);

  joints = {0.040, 0.040};
  
  joint_names = {"panda1_finger_joint1", "panda1_finger_joint2"};
  move_group_gripper.setJointValueTarget(joint_names,joints);
  success = static_cast<bool>(move_group_gripper.plan(my_plan));
  RCLCPP_INFO(LOGGER, " (gripper On) %s", success ? "" : "FAILED");
  if(success == true){
      //move_group.move();
      move_group_gripper.execute(my_plan);

  }

  sleep(1);

  
  quat_tf.setRPY(-3.142,0.0,0.0);
    quat_tf.normalize();
  target_pose1.orientation.x = quat_tf.x();
  target_pose1.orientation.y = quat_tf.y();
  target_pose1.orientation.z = quat_tf.z();
  target_pose1.orientation.w = quat_tf.w();


  target_pose1.position.x = 0.623;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.181;
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(target_pose1);

 

  success = static_cast<bool>(move_group.plan(my_plan));
              RCLCPP_INFO(LOGGER, " (movement) %s", success ? "" : "FAILED");
              if(success == true){
                  //move_group.move();
                  move_group.execute(my_plan);
                  

                  
              }
  
  sleep(1);

  quat_tf.setRPY(-3.142,0.0,0.0);
    quat_tf.normalize();
  target_pose1.orientation.x = quat_tf.x();
  target_pose1.orientation.y = quat_tf.y();
  target_pose1.orientation.z = quat_tf.z();
  target_pose1.orientation.w = quat_tf.w();


  target_pose1.position.x = 0.623;
  target_pose1.position.y = -0.1;
  target_pose1.position.z = 0.181;
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(target_pose1);

 

  success = static_cast<bool>(move_group.plan(my_plan));
              RCLCPP_INFO(LOGGER, " (movement) %s", success ? "" : "FAILED");
              if(success == true){
                  //move_group.move();
                  move_group.execute(my_plan);
                  

                  
              }
  
  sleep(1);

  quat_tf.setRPY(-3.142,0.0,0.0);
    quat_tf.normalize();
  target_pose1.orientation.x = quat_tf.x();
  target_pose1.orientation.y = quat_tf.y();
  target_pose1.orientation.z = quat_tf.z();
  target_pose1.orientation.w = quat_tf.w();


  target_pose1.position.x = 0.623;
  target_pose1.position.y = -0.1;
  target_pose1.position.z = 0.281;
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(target_pose1);

 

  success = static_cast<bool>(move_group.plan(my_plan));
              RCLCPP_INFO(LOGGER, " (movement) %s", success ? "" : "FAILED");
              if(success == true){
                  //move_group.move();
                  move_group.execute(my_plan);
                  

                  
              }
  }
  
  // sleep(3);

  // geometry_msgs::msg::TransformStamped target_pose; //= tf_buffer_.lookupTransform("tool_link","crust_base_link");
      
  //     try {
  //         //target_pose = tf_buffer_->lookupTransform();
  //         std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  //         target_pose = tf_buffer_->lookupTransform(
  //           "base_link", "aruco",
  //           tf2::TimePointZero);
  //         auto const LOGGER = rclcpp::get_logger("moveToObject");
  //         RCLCPP_INFO(LOGGER,"x: %f", target_pose.transform.translation.x);
  //         RCLCPP_INFO(LOGGER,"y: %f", target_pose.transform.translation.y);
  //         RCLCPP_INFO(LOGGER,"z: %f", target_pose.transform.translation.z);
  //         RCLCPP_INFO(LOGGER,"x: %f", target_pose.transform.rotation.x);
  //         RCLCPP_INFO(LOGGER,"y: %f", target_pose.transform.rotation.y);
  //         RCLCPP_INFO(LOGGER,"z: %f", target_pose.transform.rotation.z);
  //         RCLCPP_INFO(LOGGER,"x: %f", target_pose.transform.rotation.w);

  //         quat_tf.setRPY(0.0,1.529,-1.571);
  //         quat_tf.normalize();
  //         target_pose1.orientation.x = quat_tf.x();
  //         target_pose1.orientation.y = quat_tf.y();
  //         target_pose1.orientation.z = quat_tf.z();
  //         target_pose1.orientation.w = quat_tf.w();


  //         target_pose1.position.x = target_pose.transform.translation.x;
  //         target_pose1.position.y = target_pose.transform.translation.y;
  //         target_pose1.position.z = target_pose.transform.translation.z;
  //         move_group.setStartStateToCurrentState();
  //         move_group.setPoseTarget(target_pose1);

        

  //         // success = static_cast<bool>(move_group.plan(my_plan));
  //         //             RCLCPP_INFO(LOGGER, " (movement) %s", success ? "" : "FAILED");
  //         //             if(success == true){
  //         //                 //move_group.move();
  //         //                 move_group.execute(my_plan);
                          

                          
  //         //             }
  //     }
  //       catch (const tf2::TransformException & ex) {
  //         RCLCPP_INFO(
  //           LOGGER, "Could not transform %s to %s: %s",
  //           "ball", "crust_base_link", ex.what());
  //         return false;
  //       }



  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}