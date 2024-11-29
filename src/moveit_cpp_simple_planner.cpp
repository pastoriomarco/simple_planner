#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>
#include <algorithm>
#include <string>

// MoveItCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

// Cartesian Interpolator
#include <moveit/robot_state/cartesian_interpolator.h>

// Trajectory Processing
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>

// Planning Scene
#include <moveit/planning_scene/planning_scene.h>

// TF2 for transformations
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace rvt = rviz_visual_tools;

// Logger
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp_simple_planner");

struct MovementConfig
{
    double velocity_scaling_factor = 0.5;
    double acceleration_scaling_factor = 0.5;
    double step_size = 0.05;
    double jump_threshold = 0.0;
    std::string smoothing_type = "iterative_parabolic";
    int max_retries = 5;
    int plan_number_target = 12;
    int plan_number_limit = 32;
};

// Compute the length of a trajectory
double computePathLength(const robot_trajectory::RobotTrajectory &trajectory)
{
    double total_length = 0.0;
    const size_t waypoint_count = trajectory.getWayPointCount();

    for (size_t i = 1; i < waypoint_count; ++i)
    {
        const moveit::core::RobotState &prev_state = trajectory.getWayPoint(i - 1);
        const moveit::core::RobotState &curr_state = trajectory.getWayPoint(i);

        std::vector<double> prev_positions;
        std::vector<double> curr_positions;
        prev_state.copyJointGroupPositions(trajectory.getGroupName(), prev_positions);
        curr_state.copyJointGroupPositions(trajectory.getGroupName(), curr_positions);

        double segment_length = 0.0;
        for (size_t j = 0; j < prev_positions.size(); ++j)
        {
            double diff = curr_positions[j] - prev_positions[j];
            segment_length += diff * diff;
        }
        total_length += std::sqrt(segment_length);
    }

    return total_length;
}

// Function to apply time parameterization
bool applyTimeParameterization(
    const robot_trajectory::RobotTrajectoryPtr &trajectory,
    const MovementConfig &config,
    const rclcpp::Logger &logger)
{
    bool time_param_success = false;
    if (config.smoothing_type == "iterative_parabolic")
    {
        trajectory_processing::IterativeParabolicTimeParameterization time_param;
        time_param_success = time_param.computeTimeStamps(*trajectory,
                                                          config.velocity_scaling_factor,
                                                          config.acceleration_scaling_factor);
    }
    else if (config.smoothing_type == "time_optimal")
    {
        trajectory_processing::TimeOptimalTrajectoryGeneration time_param;
        time_param_success = time_param.computeTimeStamps(*trajectory,
                                                          config.velocity_scaling_factor,
                                                          config.acceleration_scaling_factor);
    }
    else if (config.smoothing_type == "iterative_spline")
    {
        trajectory_processing::IterativeSplineParameterization time_param;
        time_param_success = time_param.computeTimeStamps(*trajectory,
                                                          config.velocity_scaling_factor,
                                                          config.acceleration_scaling_factor);
    }
    else if (config.smoothing_type == "ruckig")
    {
        time_param_success = trajectory_processing::RuckigSmoothing::applySmoothing(
            *trajectory,
            config.velocity_scaling_factor,
            config.acceleration_scaling_factor);
    }
    else
    {
        RCLCPP_WARN(logger, "Unknown smoothing type '%s'. No time parameterization applied.", config.smoothing_type.c_str());
        time_param_success = true; // Proceed without time parameterization
    }

    if (!time_param_success)
    {
        RCLCPP_ERROR(logger, "Failed to compute time stamps for the trajectory using '%s' smoothing.",
                     config.smoothing_type.c_str());
    }

    return time_param_success;
}

bool moveToPoseTarget(
    const moveit_cpp::MoveItCppPtr &moveit_cpp_ptr,
    const moveit_cpp::PlanningComponentPtr &planning_components,
    const geometry_msgs::msg::Pose &target_pose,
    const MovementConfig &config,
    const rclcpp::Logger &logger,
    const std::string &BASE_FRAME,
    const std::string &TCP_FRAME)
{
    // Set the start state to the current state
    planning_components->setStartStateToCurrentState();

    // Create a PoseStamped with the target pose
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose = target_pose;
    pose_stamped.header.frame_id = BASE_FRAME;

    // Specify the end-effector link name
    planning_components->setGoal(pose_stamped, TCP_FRAME);

    std::vector<std::pair<moveit_cpp::PlanningComponent::PlanSolution, double>> trajectories;
    int attempts = 0;

    while (attempts < config.plan_number_limit &&
           static_cast<int>(trajectories.size()) < config.plan_number_target)
    {
        auto plan_solution = planning_components->plan();

        if (plan_solution.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
            double path_length = computePathLength(*plan_solution.trajectory);
            trajectories.emplace_back(plan_solution, path_length);
        }
        else
        {
            RCLCPP_WARN(logger, "Pose target planning attempt %d failed.", attempts + 1);
        }
        attempts++;
    }

    if (trajectories.empty())
    {
        RCLCPP_ERROR(logger, "Failed to compute any valid pose target trajectory.");
        return false;
    }

    RCLCPP_INFO(logger, "Computed %zu trajectories in %d attempts.", trajectories.size(), attempts);

    // Select the shortest trajectory
    auto shortest_trajectory_pair = std::min_element(
        trajectories.begin(), trajectories.end(),
        [](const auto &a, const auto &b)
        { return a.second < b.second; });

    // Apply time parameterization
    if (!applyTimeParameterization(shortest_trajectory_pair->first.trajectory, config, logger))
    {
        return false;
    }

    // Execute the trajectory using MoveItCpp
    int retry_count = 0;
    bool execution_success = false;
    while (retry_count < config.max_retries && !execution_success)
    {
        execution_success = moveit_cpp_ptr->execute(
            planning_components->getPlanningGroupName(), shortest_trajectory_pair->first.trajectory);
        if (execution_success)
        {
            // Wait for the execution to complete
            auto tem = moveit_cpp_ptr->getTrajectoryExecutionManager();
            bool wait_success = tem->waitForExecution();

            // Check if execution completed successfully
            if (!wait_success || tem->getLastExecutionStatus() != moveit_controller_manager::ExecutionStatus::SUCCEEDED)
            {
                RCLCPP_WARN(logger, "Execution failed to complete successfully.");
                execution_success = false;
            }
        }
        else
        {
            RCLCPP_WARN(logger, "Execution failed, retrying...");
        }
        retry_count++;
    }

    if (!execution_success)
    {
        RCLCPP_ERROR(logger, "Execution failed after retries.");
        return false;
    }

    return true;
}

bool moveToJointTarget(
    const moveit_cpp::MoveItCppPtr &moveit_cpp_ptr,
    const moveit_cpp::PlanningComponentPtr &planning_components,
    const std::vector<double> &joint_values,
    const MovementConfig &config,
    const rclcpp::Logger &logger)
{
    // Set the start state to the current state
    planning_components->setStartStateToCurrentState();

    // Create a RobotState and set joint positions
    moveit::core::RobotState goal_state(*moveit_cpp_ptr->getCurrentState());
    goal_state.setJointGroupPositions(planning_components->getPlanningGroupName(), joint_values);
    goal_state.update();

    planning_components->setGoal(goal_state);

    std::vector<std::pair<moveit_cpp::PlanningComponent::PlanSolution, double>> trajectories;
    int attempts = 0;

    while (attempts < config.plan_number_limit &&
           static_cast<int>(trajectories.size()) < config.plan_number_target)
    {
        auto plan_solution = planning_components->plan();

        if (plan_solution.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
            double path_length = computePathLength(*plan_solution.trajectory);
            trajectories.emplace_back(plan_solution, path_length);
        }
        else
        {
            RCLCPP_WARN(logger, "Joint target planning attempt %d failed.", attempts + 1);
        }
        attempts++;
    }

    if (trajectories.empty())
    {
        RCLCPP_ERROR(logger, "Failed to compute any valid joint target trajectory.");
        return false;
    }

    RCLCPP_INFO(logger, "Computed %zu trajectories in %d attempts.", trajectories.size(), attempts);

    // Select the shortest trajectory
    auto shortest_trajectory_pair = std::min_element(
        trajectories.begin(), trajectories.end(),
        [](const auto &a, const auto &b)
        { return a.second < b.second; });

    // Apply time parameterization
    if (!applyTimeParameterization(shortest_trajectory_pair->first.trajectory, config, logger))
    {
        return false;
    }

    // Execute the trajectory using MoveItCpp
    int retry_count = 0;
    bool execution_success = false;
    while (retry_count < config.max_retries && !execution_success)
    {
        execution_success = moveit_cpp_ptr->execute(
            planning_components->getPlanningGroupName(), shortest_trajectory_pair->first.trajectory);
        if (execution_success)
        {
            // Wait for the execution to complete
            auto tem = moveit_cpp_ptr->getTrajectoryExecutionManager();
            bool wait_success = tem->waitForExecution();

            // Check if execution completed successfully
            if (!wait_success || tem->getLastExecutionStatus() != moveit_controller_manager::ExecutionStatus::SUCCEEDED)
            {
                RCLCPP_WARN(logger, "Execution failed to complete successfully.");
                execution_success = false;
            }
        }
        else
        {
            RCLCPP_WARN(logger, "Execution failed, retrying...");
        }
        retry_count++;
    }

    if (!execution_success)
    {
        RCLCPP_ERROR(logger, "Execution failed after retries.");
        return false;
    }

    return true;
}

bool moveToNamedTarget(
    const moveit_cpp::MoveItCppPtr &moveit_cpp_ptr,
    const moveit_cpp::PlanningComponentPtr &planning_components,
    const std::string &target_name,
    const MovementConfig &config,
    const rclcpp::Logger &logger)
{
    // Set the start state to the current state
    planning_components->setStartStateToCurrentState();

    planning_components->setGoal(target_name);

    std::vector<std::pair<moveit_cpp::PlanningComponent::PlanSolution, double>> trajectories;
    int attempts = 0;

    while (attempts < config.plan_number_limit &&
           static_cast<int>(trajectories.size()) < config.plan_number_target)
    {
        auto plan_solution = planning_components->plan();

        if (plan_solution.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
            double path_length = computePathLength(*plan_solution.trajectory);
            trajectories.emplace_back(plan_solution, path_length);
        }
        else
        {
            RCLCPP_WARN(logger, "Named target planning attempt %d failed.", attempts + 1);
        }
        attempts++;
    }

    if (trajectories.empty())
    {
        RCLCPP_ERROR(logger, "Failed to compute any valid named target trajectory.");
        return false;
    }

    RCLCPP_INFO(logger, "Computed %zu trajectories in %d attempts.", trajectories.size(), attempts);

    // Select the shortest trajectory
    auto shortest_trajectory_pair = std::min_element(
        trajectories.begin(), trajectories.end(),
        [](const auto &a, const auto &b)
        { return a.second < b.second; });

    // Apply time parameterization
    if (!applyTimeParameterization(shortest_trajectory_pair->first.trajectory, config, logger))
    {
        return false;
    }

    // Execute the trajectory using MoveItCpp
    int retry_count = 0;
    bool execution_success = false;
    while (retry_count < config.max_retries && !execution_success)
    {
        execution_success = moveit_cpp_ptr->execute(
            planning_components->getPlanningGroupName(), shortest_trajectory_pair->first.trajectory);
        if (execution_success)
        {
            // Wait for the execution to complete
            auto tem = moveit_cpp_ptr->getTrajectoryExecutionManager();
            bool wait_success = tem->waitForExecution();

            // Check if execution completed successfully
            if (!wait_success || tem->getLastExecutionStatus() != moveit_controller_manager::ExecutionStatus::SUCCEEDED)
            {
                RCLCPP_WARN(logger, "Execution failed to complete successfully.");
                execution_success = false;
            }
        }
        else
        {
            RCLCPP_WARN(logger, "Execution failed, retrying...");
        }
        retry_count++;
    }

    if (!execution_success)
    {
        RCLCPP_ERROR(logger, "Execution failed after retries.");
        return false;
    }

    return true;
}

bool moveCartesianPath(
    const moveit_cpp::MoveItCppPtr &moveit_cpp_ptr,
    const moveit_cpp::PlanningComponentPtr &planning_components,
    const std::vector<geometry_msgs::msg::Pose> &waypoints,
    const MovementConfig &config,
    const rclcpp::Logger &logger,
    const double linear_success_tolerance = 0.99,
    const std::string &TCP_FRAME = "link_tcp")
{
    // Get the robot model and joint model group
    auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
    auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(planning_components->getPlanningGroupName());

    // Specify the end-effector link name
    const moveit::core::LinkModel *link_model = robot_model_ptr->getLinkModel(TCP_FRAME);

    std::vector<std::pair<robot_trajectory::RobotTrajectoryPtr, double>> trajectories;
    int attempts = 0;

    while (attempts < config.plan_number_limit &&
           static_cast<int>(trajectories.size()) < config.plan_number_target)
    {
        // Update current state before each attempt
        auto current_state = moveit_cpp_ptr->getCurrentState();

        std::vector<moveit::core::RobotStatePtr> trajectory_states;

        // Convert waypoints to EigenSTL::vector_Isometry3d
        EigenSTL::vector_Isometry3d waypoints_eigen;
        for (const auto &pose_msg : waypoints)
        {
            Eigen::Isometry3d pose_eigen;
            tf2::fromMsg(pose_msg, pose_eigen);
            waypoints_eigen.push_back(pose_eigen);
        }

        moveit::core::MaxEEFStep max_step(config.step_size, 0.0);
        moveit::core::JumpThreshold jump_threshold(config.jump_threshold, config.jump_threshold);

        // Define the validity callback
        moveit::core::GroupStateValidityCallbackFn validity_callback =
            [moveit_cpp_ptr](moveit::core::RobotState *state, const moveit::core::JointModelGroup *group,
                             const double *joint_group_variable_values)
        {
            state->setJointGroupPositions(group, joint_group_variable_values);
            state->update();
            return !planning_scene_monitor::LockedPlanningSceneRO(moveit_cpp_ptr->getPlanningSceneMonitor())->isStateColliding(*state, group->getName());
        };

        double achieved_fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
            current_state.get(), joint_model_group_ptr, trajectory_states, link_model,
            waypoints_eigen, true /* global_reference_frame */, max_step, jump_threshold,
            validity_callback);

        if (achieved_fraction >= linear_success_tolerance)
        {
            // Create a RobotTrajectory
            auto trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_ptr, planning_components->getPlanningGroupName());

            // Add the waypoints to the trajectory
            for (size_t i = 0; i < trajectory_states.size(); ++i)
            {
                trajectory->addSuffixWayPoint(trajectory_states[i], 0.0); // Time will be updated
            }

            double path_length = computePathLength(*trajectory);
            trajectories.emplace_back(trajectory, path_length);
        }
        else
        {
            RCLCPP_WARN(logger, "Cartesian path planning attempt %d failed (%.2f%% achieved).", attempts + 1, achieved_fraction * 100.0);
        }
        attempts++;
    }

    if (trajectories.empty())
    {
        RCLCPP_ERROR(logger, "Failed to compute any valid Cartesian path.");
        return false;
    }

    RCLCPP_INFO(logger, "Computed %zu trajectories in %d attempts.", trajectories.size(), attempts);

    // Select the shortest trajectory
    auto shortest_trajectory_pair = std::min_element(
        trajectories.begin(), trajectories.end(),
        [](const auto &a, const auto &b)
        { return a.second < b.second; });

    // Apply time parameterization
    if (!applyTimeParameterization(shortest_trajectory_pair->first, config, logger))
    {
        return false;
    }

    // Execute the trajectory using MoveItCpp
    int retry_count = 0;
    bool execution_success = false;
    while (retry_count < config.max_retries && !execution_success)
    {
        execution_success = moveit_cpp_ptr->execute(
            planning_components->getPlanningGroupName(), shortest_trajectory_pair->first);
        if (execution_success)
        {
            // Wait for the execution to complete
            auto tem = moveit_cpp_ptr->getTrajectoryExecutionManager();
            bool wait_success = tem->waitForExecution();

            // Check if execution completed successfully
            if (!wait_success || tem->getLastExecutionStatus() != moveit_controller_manager::ExecutionStatus::SUCCEEDED)
            {
                RCLCPP_WARN(logger, "Execution failed to complete successfully.");
                execution_success = false;
            }
        }
        else
        {
            RCLCPP_WARN(logger, "Execution failed, retrying...");
        }
        retry_count++;
    }

    if (!execution_success)
    {
        RCLCPP_ERROR(logger, "Execution failed after retries.");
        return false;
    }

    return true;
}

int main(int argc, char **argv)
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    RCLCPP_INFO(LOGGER, "Initialize node");

    node_options.automatically_declare_parameters_from_overrides(true);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("run_moveit_cpp", "", node_options);
    // Create a ROS logger
    auto logger = node->get_logger();

    // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]()
                               { executor.spin(); });

    // Get parameters
    std::string robot_type;
    node->get_parameter_or<std::string>("robot_type", robot_type, "lite6");
    std::string BASE_FRAME;
    node->get_parameter_or<std::string>("base_link", BASE_FRAME, "link_base");
    std::string TCP_FRAME;
    node->get_parameter_or<std::string>("tcp_frame", TCP_FRAME, "link_tcp");

    MovementConfig std_move_config;
    node->get_parameter_or<double>("velocity_scaling_factor", std_move_config.velocity_scaling_factor, 0.9);
    node->get_parameter_or<double>("acceleration_scaling_factor", std_move_config.acceleration_scaling_factor, 0.9);
    node->get_parameter_or<int>("max_retries", std_move_config.max_retries, 5);
    node->get_parameter_or<std::string>("smoothing_type", std_move_config.smoothing_type, "iterative_parabolic");
    node->get_parameter_or<double>("step_size", std_move_config.step_size, 0.05);
    node->get_parameter_or<double>("jump_threshold", std_move_config.jump_threshold, 0.0);
    node->get_parameter_or<int>("plan_number_target", std_move_config.plan_number_target, 12);
    node->get_parameter_or<int>("plan_number_limit", std_move_config.plan_number_limit, 32);

    MovementConfig slow_move_config = std_move_config;
    slow_move_config.velocity_scaling_factor = 0.1;
    slow_move_config.acceleration_scaling_factor = 0.1;

    // Create the MoveItCpp instance
    auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(node);
    moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();

    // Create the PlanningComponent
    auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(robot_type, moveit_cpp_ptr);

    RCLCPP_INFO_STREAM(LOGGER, "Set smoothing type: " << std_move_config.smoothing_type);
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Create a collision object for the robot to avoid
    auto collision_object = [BASE_FRAME]()
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = BASE_FRAME;
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

    {
        // Add object to planning scene
        planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_ptr->getPlanningSceneMonitor());
        scene->processCollisionObjectMsg(collision_object);
    }

    // Set a target Pose
    auto approach_pose = []()
    {
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

    // Move to joint target
    do
    {
        result = moveToJointTarget(moveit_cpp_ptr, planning_components, ready_joint_values, std_move_config, logger);
    } while (!result);

    // Move to pose target
    do
    {
        result = moveToPoseTarget(moveit_cpp_ptr, planning_components, approach_pose, std_move_config, logger, BASE_FRAME, TCP_FRAME);
    } while (!result);

    // Move to Cartesian path (grasp)
    do
    {
        result = moveCartesianPath(moveit_cpp_ptr, planning_components, grasp_waypoints, slow_move_config, logger, 0.99, TCP_FRAME);
    } while (!result);

    // Retract to Cartesian path (approach)
    do
    {
        result = moveCartesianPath(moveit_cpp_ptr, planning_components, approach_waypoints, slow_move_config, logger, 0.99, TCP_FRAME);
    } while (!result);

    // Move to named target ("home")
    do
    {
        result = moveToNamedTarget(moveit_cpp_ptr, planning_components, "home", std_move_config, logger);
    } while (!result);

    // Shutdown ROS
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
