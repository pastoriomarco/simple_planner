#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

// N.D. Marco Pastorio:
// Simple planner implemented using MoveGroupInterface

struct MovementConfig {
  double velocity_scaling_factor = 0.5;
  double acceleration_scaling_factor = 0.5;
  double step_size = 0.05;
  double jump_threshold = 0.0;
  std::string smoothing_type = "time_optimal";
  int max_exec_retries = 5;
  int plan_number_target = 12;
  int plan_number_limit = 32;
};

double computePathLength(const moveit_msgs::msg::RobotTrajectory &trajectory) {
  double total_length = 0.0;
  const auto &joint_trajectory = trajectory.joint_trajectory;

  for (size_t i = 1; i < joint_trajectory.points.size(); ++i) {
    double segment_length = 0.0;

    for (size_t j = 0; j < joint_trajectory.points[i].positions.size(); ++j) {
      double diff = joint_trajectory.points[i].positions[j] - joint_trajectory.points[i - 1].positions[j];
      segment_length += diff * diff;
    }
    total_length += std::sqrt(segment_length);
  }
  return total_length;
}

bool moveCartesianPath(
    moveit::planning_interface::MoveGroupInterface &move_group_interface,
    const std::vector<geometry_msgs::msg::Pose> &waypoints,
    const MovementConfig &config,
    const std::shared_ptr<rclcpp::Logger> &logger,
    const double linear_success_tolerance = 0.99) {

  // Retry variables
  int retry_count = 0;
  bool smoothing_success = false;

  std::vector<geometry_msgs::msg::Pose> full_waypoints;
  moveit_msgs::msg::RobotTrajectory trajectory_msg;

  // Always add the current pose as the start pose
  full_waypoints.push_back(move_group_interface.getCurrentPose().pose);
  full_waypoints.insert(full_waypoints.end(), waypoints.begin(), waypoints.end());

  std::vector<std::pair<moveit_msgs::msg::RobotTrajectory, double>> trajectories;
  int attempts = 0;

  while (attempts < config.plan_number_limit && trajectories.size() < static_cast<size_t>(config.plan_number_target)) {
    double fraction = move_group_interface.computeCartesianPath(
        full_waypoints, config.step_size, config.jump_threshold, trajectory_msg);

    if (fraction >= linear_success_tolerance) {
      double path_length = computePathLength(trajectory_msg);
      trajectories.emplace_back(trajectory_msg, path_length);
    } else {
      RCLCPP_WARN(*logger, "Cartesian path planning attempt %d failed (%.2f%% achieved).", attempts + 1, fraction * 100.0);
    }
    attempts++;
  }

  if (trajectories.empty()) {
    RCLCPP_ERROR(*logger, "Failed to compute any valid Cartesian path.");
    return false;
  } else {
    RCLCPP_INFO_STREAM(*logger, "Computed " << trajectories.size() << " trajectories in " << attempts << " attempts.");
  }

  // Select the shortest trajectory
  auto shortest_trajectory_pair = std::min_element(
      trajectories.begin(), trajectories.end(),
      [](const auto &a, const auto &b) { return a.second < b.second; });

  trajectory_msg = shortest_trajectory_pair->first;

  // Convert to trajectory for smoothing
  robot_trajectory::RobotTrajectory trajectory(
      move_group_interface.getRobotModel(), move_group_interface.getName());
  trajectory.setRobotTrajectoryMsg(*move_group_interface.getCurrentState(), trajectory_msg);

  // Apply smoothing as per config
  while ((retry_count <= config.max_exec_retries) && (!smoothing_success)) {
    if (config.smoothing_type == "time_optimal") {
      trajectory_processing::TimeOptimalTrajectoryGeneration time_param;
      smoothing_success = time_param.computeTimeStamps(
          trajectory, config.velocity_scaling_factor, config.acceleration_scaling_factor);
    } else if (config.smoothing_type == "iterative") {
      trajectory_processing::IterativeSplineParameterization time_param;
      smoothing_success = time_param.computeTimeStamps(
          trajectory, config.velocity_scaling_factor, config.acceleration_scaling_factor);
    } else {
      smoothing_success = trajectory_processing::RuckigSmoothing::applySmoothing(
          trajectory, config.velocity_scaling_factor, config.acceleration_scaling_factor);
    }
    retry_count++;
  }

  if (!smoothing_success) {
    RCLCPP_WARN(*logger, "Failed to apply smoothing.");
    return false;
  }

  retry_count = 0;
  trajectory.getRobotTrajectoryMsg(trajectory_msg);

  // Apply scaling factors during execution
  move_group_interface.setMaxVelocityScalingFactor(config.velocity_scaling_factor);
  move_group_interface.setMaxAccelerationScalingFactor(config.acceleration_scaling_factor);

  while (retry_count < config.max_exec_retries) {
    if (move_group_interface.execute(trajectory_msg) == moveit::core::MoveItErrorCode::SUCCESS) {
      return true;
    }
    RCLCPP_WARN(*logger, "Execution failed, retrying...");
    retry_count++;
  }

  RCLCPP_ERROR(*logger, "Execution failed after retries.");
  return false;
}

bool moveToPoseTarget(
    moveit::planning_interface::MoveGroupInterface& move_group_interface,
    const geometry_msgs::msg::Pose& target_pose,
    const MovementConfig& config,
    const std::shared_ptr<rclcpp::Logger>& logger) {

  move_group_interface.setPoseTarget(target_pose);

  // Apply scaling factors
  move_group_interface.setMaxVelocityScalingFactor(config.velocity_scaling_factor);
  move_group_interface.setMaxAccelerationScalingFactor(config.acceleration_scaling_factor);

  std::vector<std::pair<moveit_msgs::msg::RobotTrajectory, double>> trajectories;
  int attempts = 0;

  while (attempts < config.plan_number_limit && trajectories.size() < static_cast<size_t>(config.plan_number_target)) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_interface.plan(plan)) {
      double path_length = computePathLength(plan.trajectory_);
      trajectories.emplace_back(plan.trajectory_, path_length);
    } else {
      RCLCPP_WARN(*logger, "Pose target planning attempt %d failed.", attempts + 1);
    }
    attempts++;
  }

  if (trajectories.empty()) {
    RCLCPP_ERROR(*logger, "Failed to compute any valid pose target trajectory.");
    return false;
  }

  auto shortest_trajectory = std::min_element(
      trajectories.begin(), trajectories.end(),
      [](const auto& a, const auto& b) { return a.second < b.second; });

  int retry_count = 0;
  while (retry_count < config.max_exec_retries) {
    if (move_group_interface.execute(shortest_trajectory->first) == moveit::core::MoveItErrorCode::SUCCESS) {
      return true;
    }
    RCLCPP_WARN(*logger, "Execution failed, retrying...");
    retry_count++;
  }

  RCLCPP_ERROR(*logger, "Execution failed after retries.");
  return false;
}

bool moveToNamedTarget(
    moveit::planning_interface::MoveGroupInterface& move_group_interface,
    const std::string& target_name,
    const MovementConfig& config,
    const std::shared_ptr<rclcpp::Logger>& logger) {

  move_group_interface.setNamedTarget(target_name);

  // Apply scaling factors
  move_group_interface.setMaxVelocityScalingFactor(config.velocity_scaling_factor);
  move_group_interface.setMaxAccelerationScalingFactor(config.acceleration_scaling_factor);

  std::vector<std::pair<moveit_msgs::msg::RobotTrajectory, double>> trajectories;
  int attempts = 0;

  while (attempts < config.plan_number_limit && trajectories.size() < static_cast<size_t>(config.plan_number_target)) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_interface.plan(plan)) {
      double path_length = computePathLength(plan.trajectory_);
      trajectories.emplace_back(plan.trajectory_, path_length);
    } else {
      RCLCPP_WARN(*logger, "Named target planning attempt %d failed.", attempts + 1);
    }
    attempts++;
  }

  if (trajectories.empty()) {
    RCLCPP_ERROR(*logger, "Failed to compute any valid named target trajectory.");
    return false;
  }

  auto shortest_trajectory = std::min_element(
      trajectories.begin(), trajectories.end(),
      [](const auto& a, const auto& b) { return a.second < b.second; });

  int retry_count = 0;
  while (retry_count < config.max_exec_retries) {
    if (move_group_interface.execute(shortest_trajectory->first) == moveit::core::MoveItErrorCode::SUCCESS) {
      return true;
    }
    RCLCPP_WARN(*logger, "Execution failed, retrying...");
    retry_count++;
  }

  RCLCPP_ERROR(*logger, "Execution failed after retries.");
  return false;
}

bool moveToJointTarget(
    moveit::planning_interface::MoveGroupInterface &move_group_interface,
    const std::vector<double> &joint_values,
    const MovementConfig &config,
    const std::shared_ptr<rclcpp::Logger> &logger) {

  move_group_interface.setJointValueTarget(joint_values);

  // Apply scaling factors
  move_group_interface.setMaxVelocityScalingFactor(config.velocity_scaling_factor);
  move_group_interface.setMaxAccelerationScalingFactor(config.acceleration_scaling_factor);

  std::vector<std::pair<moveit_msgs::msg::RobotTrajectory, double>> trajectories;
  int attempts = 0;

  while (attempts < config.plan_number_limit && trajectories.size() < static_cast<size_t>(config.plan_number_target)) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_interface.plan(plan)) {
      double path_length = computePathLength(plan.trajectory_);
      trajectories.emplace_back(plan.trajectory_, path_length);
    } else {
      RCLCPP_WARN(*logger, "Joint target planning attempt %d failed.", attempts + 1);
    }
    attempts++;
  }

  if (trajectories.empty()) {
    RCLCPP_ERROR(*logger, "Failed to compute any valid joint target trajectory.");
    return false;
  } else {
    RCLCPP_INFO_STREAM(*logger, "Computed " << trajectories.size() << " trajectories in " << attempts << " attempts.");
  }

  // Select the shortest trajectory
  auto shortest_trajectory_pair = std::min_element(
      trajectories.begin(), trajectories.end(),
      [](const auto &a, const auto &b) { return a.second < b.second; });

  int retry_count = 0;
  while (retry_count < config.max_exec_retries) {
    if (move_group_interface.execute(shortest_trajectory_pair->first) == moveit::core::MoveItErrorCode::SUCCESS) {
      return true;
    }
    RCLCPP_WARN(*logger, "Execution failed, retrying...");
    retry_count++;
  }

  RCLCPP_ERROR(*logger, "Execution failed after retries.");
  return false;
}

int main(int argc, char *argv[]) {
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "simple_planner",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("simple_planner"));

  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Get parameters
  std::string robot_type;
  node->get_parameter_or<std::string>("robot_type", robot_type, "lite6");
  std::string base_link;
  node->get_parameter_or<std::string>("base_link", base_link, "link_base");

  MovementConfig std_move_config;
  node->get_parameter_or<double>("velocity_scaling_factor", std_move_config.velocity_scaling_factor, 0.9);
  node->get_parameter_or<double>("acceleration_scaling_factor", std_move_config.acceleration_scaling_factor, 0.9);
  node->get_parameter_or<int>("max_exec_retries", std_move_config.max_exec_retries, 5);
  node->get_parameter_or<std::string>("smoothing_type", std_move_config.smoothing_type, "time_optimal");
  node->get_parameter_or<double>("step_size", std_move_config.step_size, 0.05);
  node->get_parameter_or<double>("jump_threshold", std_move_config.jump_threshold, 0.0);
  node->get_parameter_or<int>("plan_number_target", std_move_config.plan_number_target, 12);
  node->get_parameter_or<int>("plan_number_limit", std_move_config.plan_number_limit, 32);

  MovementConfig slow_move_config = std_move_config;
  slow_move_config.velocity_scaling_factor = 0.1;
  slow_move_config.acceleration_scaling_factor = 0.1;

  // Create the MoveIt MoveGroup Interface
  moveit::planning_interface::MoveGroupInterface move_group_interface(node, robot_type);

  auto current_planner = move_group_interface.getPlannerId();
  RCLCPP_INFO(*logger, "Using planner ID: %s", current_planner.c_str());

  // Create a collision object for the robot to avoid
  auto const collision_object = [frame_id = move_group_interface.getPlanningFrame()]() {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box1";

    shape_msgs::msg::SolidPrimitive primitive;
    // Define size of box
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.5;
    primitive.dimensions[primitive.BOX_Y] = 0.05;
    primitive.dimensions[primitive.BOX_Z] = 0.3;

    // Define pose of box (relative to frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.15;
    box_pose.position.y = 0.2;
    box_pose.position.z = 0.15;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);

  // Set a target Pose
  auto const approach_pose = []() {
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = 1.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = 0.0;
    msg.position.x = 0.1;
    msg.position.y = 0.3;
    msg.position.z = 0.2;
    return msg;
  }();

  // Define waypoints for Cartesian path
  std::vector<geometry_msgs::msg::Pose> grasp_waypoints;
  std::vector<geometry_msgs::msg::Pose> approach_waypoints;

  // Target pose (straight-line path)
  geometry_msgs::msg::Pose grasp_pose = approach_pose;
  grasp_pose.position.z -= 0.05; // Move 5 cm along Z-axis
  grasp_waypoints.push_back(grasp_pose);

  approach_waypoints.push_back(approach_pose);

  // Joint target
  std::vector<double> ready_joint_values = {0.0, 0.0, 1.57, 0.0, 0.0, 0.0};

  // MOVEMENTS SEQUENCE
  bool result = false;

  do {
    result = moveToJointTarget(move_group_interface, ready_joint_values, std_move_config, logger);
  } while (!result);

  // Move to pose target
  do {
    result = moveToPoseTarget(move_group_interface, approach_pose, std_move_config, logger);
  } while (!result);

  // Move to Cartesian path
  do {
    result = moveCartesianPath(move_group_interface, grasp_waypoints, slow_move_config, logger);
  } while (!result);

  // Retract to Cartesian path
  do {
    result = moveCartesianPath(move_group_interface, approach_waypoints, slow_move_config, logger);
  } while (!result);

  // Move to named target ("home")
  do {
    result = moveToNamedTarget(move_group_interface, "home", std_move_config, logger);
  } while (!result);

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
