#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <iostream>
using moveit::planning_interface::MoveGroupInterface;
int main(int argc, char * argv[])
{
// Initialize ROS and create the Node
rclcpp::init(argc, argv);
auto const node = std::make_shared<rclcpp::Node>("rob_manip_move",
rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
);
// Set use_sim_time if using Gazebo simulation
std::vector<rclcpp::Parameter> parameters{rclcpp::Parameter("use_sim_time", true)};
node->set_parameters(parameters);
// We spin up a SingleThreadedExecutor for the current state monitor to get information about the robot's state.
rclcpp::executors::SingleThreadedExecutor executor;
executor.add_node(node);
std::thread([&executor]() { executor.spin(); }).detach();
// Create a ROS logger
auto const logger = rclcpp::get_logger("rob_manip_move");
// Create the MoveIt MoveGroup Interface and start state monitor
auto move_group_interface = MoveGroupInterface(node, "arm");
move_group_interface.startStateMonitor();
// Set a target Pose
geometry_msgs::msg::Pose target_pose;
target_pose.position.x = 0.38;
target_pose.position.y = 0.0;
target_pose.position.z = 0.2;
tf2::Quaternion q;
q.setRPY(0.0, 3.14, 0.0);
q=q.normalize();
target_pose.orientation.x = q.getX();
target_pose.orientation.y = q.getY();
target_pose.orientation.z = q.getZ();
target_pose.orientation.w = q.getW();
move_group_interface.setPoseTarget(target_pose);
// Create a plan to that target pose
moveit::planning_interface::MoveGroupInterface::Plan plan;
auto success = static_cast<bool>(move_group_interface.plan(plan));
// Execute the plan
if(success) {
move_group_interface.execute(plan);
} else {
RCLCPP_ERROR(logger, "Planning failed!");
}
// Create collision object for the robot to avoid
moveit_msgs::msg::CollisionObject collision_object;
collision_object.header.frame_id = move_group_interface.getPlanningFrame();
collision_object.id = "box";
shape_msgs::msg::SolidPrimitive primitive;
// Define the size of the box
primitive.type = primitive.BOX;
primitive.dimensions.resize(3);
primitive.dimensions[primitive.BOX_X] = 0.3;
primitive.dimensions[primitive.BOX_Y] = 0.1;
primitive.dimensions[primitive.BOX_Z] = 0.3;
// Define the pose of the box (relative to the frame_id)
geometry_msgs::msg::Pose box_pose;
box_pose.orientation.w = 1.0; // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
box_pose.position.x = 0.35;
box_pose.position.y = 0.25;
box_pose.position.z = 0.25;
collision_object.primitives.push_back(primitive);
collision_object.primitive_poses.push_back(box_pose);
collision_object.operation = collision_object.ADD;
// Add the collision object to the scene
moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
planning_scene_interface.applyCollisionObject(collision_object);
//Pause and request that user activate next move command
std::cout << "Press any key!" << std::endl;
std::cin.get();
// RobotState is the object that contains all the current position/velocity/acceleration data.
moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState(10);
// Get the current set of joint values.
std::vector<double> joint_group_positions;
current_state->copyJointGroupPositions("arm", joint_group_positions);
// Modify the first joint, by rotating 90 degrees
joint_group_positions[0] = 1.57; // radians
move_group_interface.setJointValueTarget(joint_group_positions);
success = static_cast<bool>(move_group_interface.plan(plan));
// Execute the plan
if(success) {
move_group_interface.execute(plan);
} else {
RCLCPP_ERROR(logger, "Planning failed!");
}
// Shutdown ROS
rclcpp::shutdown();
return 0;
}