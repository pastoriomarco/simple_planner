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
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

// Planning Scene
#include <moveit/planning_scene/planning_scene.h>

// TF2 for transformations
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>

// #include <moveit_visual_tools/moveit_visual_tools.h>
// namespace rvt = rviz_visual_tools;

struct MovementConfig
{
    double velocity_scaling_factor = 0.5;
    double acceleration_scaling_factor = 0.5;
    double step_size = 0.01;
    double jump_threshold = 0.0;
    double max_cartesian_speed = 0.5;
    int max_exec_retries = 5;
    int plan_number_target = 12;
    int plan_number_limit = 32;
    std::string smoothing_type = "iterative_parabolic";
};

// Compute the length of a trajectory considering joint movements
double computeJointPathLength(const robot_trajectory::RobotTrajectory &trajectory)
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

// Compute the length of a trajectory considering TCP linear movements
double computeCartesianPathLength(const robot_trajectory::RobotTrajectory &trajectory, const std::string &tcp_frame)
{
    double total_length = 0.0;
    const size_t waypoint_count = trajectory.getWayPointCount();

    for (size_t i = 1; i < waypoint_count; ++i)
    {
        const moveit::core::RobotState &prev_state = trajectory.getWayPoint(i - 1);
        const moveit::core::RobotState &curr_state = trajectory.getWayPoint(i);

        Eigen::Isometry3d prev_pose = prev_state.getGlobalLinkTransform(tcp_frame);
        Eigen::Isometry3d curr_pose = curr_state.getGlobalLinkTransform(tcp_frame);

        double distance = (curr_pose.translation() - prev_pose.translation()).norm();
        total_length += distance;
    }

    return total_length;
}

// Compute the length of a trajectory considering joint movements
double computePathLength(const robot_trajectory::RobotTrajectory &trajectory, const std::string &tcp_frame)
{
    //
    // // Create a Clock object using System Time
    // rclcpp::Clock clock(RCL_ROS_TIME);

    // rclcpp::Time now = clock.now();
    // double timestamp = now.seconds();

    double joint_length = computeJointPathLength(trajectory);

    // double joint_time = timestamp;
    // now = clock.now();
    // timestamp = now.seconds();
    // joint_time = timestamp - joint_time;

    double cartesian_length = computeCartesianPathLength(trajectory, tcp_frame);

    // double cartesian_time = timestamp;
    // now = clock.now();
    // timestamp = now.seconds();
    // cartesian_time = timestamp - cartesian_time;

    // RCLCPP_INFO_STREAM(logger, "Joint lenght: " << joint_length / 2 << ", computational time: " << joint_time);
    // RCLCPP_INFO_STREAM(logger, "Cartesian lenght: " << 2 * cartesian_length << ", computational time: " << cartesian_time);

    double total_length = (2 * cartesian_length + joint_length / 2);

    // RCLCPP_INFO_STREAM(logger, "Path lenght: " << total_length);

    return total_length;
}

// Function to compute maximum Cartesian speed in the trajectory
double computeMaxCartesianSpeed(
    const robot_trajectory::RobotTrajectoryPtr &trajectory,
    const std::string &tcp_frame)
{
    const size_t waypoint_count = trajectory->getWayPointCount();
    if (waypoint_count < 2)
    {
        return 0.0;
    }

    double max_speed = 0.0;

    for (size_t i = 1; i < waypoint_count; ++i)
    {
        auto prev_state = trajectory->getWayPointPtr(i - 1);
        auto curr_state = trajectory->getWayPointPtr(i);

        // Get TCP positions
        const Eigen::Isometry3d prev_pose = prev_state->getGlobalLinkTransform(tcp_frame);
        const Eigen::Isometry3d curr_pose = curr_state->getGlobalLinkTransform(tcp_frame);

        // Compute Cartesian distance
        double distance = (curr_pose.translation() - prev_pose.translation()).norm();

        // Get duration between waypoints
        double duration = trajectory->getWayPointDurationFromPrevious(i);

        // Compute Cartesian speed
        double cartesian_speed = distance / duration;

        if (cartesian_speed > max_speed)
        {
            max_speed = cartesian_speed;
        }
    }

    return max_speed;
}

// Function to apply time parameterization with adjusted scaling factors
bool applyTimeParameterization(
    const robot_trajectory::RobotTrajectoryPtr &trajectory,
    const MovementConfig &config,
    const std::string &tcp_frame,
    const rclcpp::Logger &logger)
{
    // Copy the scaling factors so we can adjust them locally
    double velocity_scaling_factor = config.velocity_scaling_factor;
    double acceleration_scaling_factor = config.acceleration_scaling_factor;

    const int max_iterations = 5;
    int iteration = 0;
    bool time_param_success = false;

    while (iteration < max_iterations)
    {
        // Remove existing timing by resetting durations to zero
        for (size_t i = 1; i < trajectory->getWayPointCount(); ++i)
        {
            trajectory->setWayPointDurationFromPrevious(i, 0.0);
        }

        // Apply time parameterization
        if (config.smoothing_type == "time_optimal")
        {
            trajectory_processing::TimeOptimalTrajectoryGeneration time_param;
            time_param_success = time_param.computeTimeStamps(*trajectory,
                                                              velocity_scaling_factor,
                                                              acceleration_scaling_factor);
        }
        else // if (config.smoothing_type == "iterative_parabolic")
        {
            trajectory_processing::IterativeParabolicTimeParameterization time_param;
            time_param_success = time_param.computeTimeStamps(*trajectory,
                                                              velocity_scaling_factor,
                                                              acceleration_scaling_factor);
        }

        if (!time_param_success)
        {
            RCLCPP_ERROR(logger, "Failed to compute time stamps for the trajectory using '%s' smoothing.",
                         config.smoothing_type.c_str());
            return false;
        }

        // Compute the maximum Cartesian speed achieved
        double max_cartesian_speed_achieved = computeMaxCartesianSpeed(trajectory, tcp_frame);

        if (max_cartesian_speed_achieved <= config.max_cartesian_speed)
        {
            // Success, the Cartesian speed is within limits
            return true;
        }
        else
        {
            // Adjust scaling factors proportionally
            double scaling_factor = config.max_cartesian_speed / max_cartesian_speed_achieved;
            velocity_scaling_factor *= scaling_factor;
            acceleration_scaling_factor = ((acceleration_scaling_factor * scaling_factor) + acceleration_scaling_factor) / 2.0;

            RCLCPP_WARN(logger, "Adjusted scaling factors to limit Cartesian speed: velocity_scaling_factor=%.3f, acceleration_scaling_factor=%.3f",
                        velocity_scaling_factor, acceleration_scaling_factor);

            // Ensure scaling factors do not become too small
            const double min_scaling_factor = 0.01;
            if (velocity_scaling_factor < min_scaling_factor || acceleration_scaling_factor < min_scaling_factor)
            {
                RCLCPP_ERROR(logger, "Scaling factors became too small. Cannot limit Cartesian speed further without violating joint limits.");
                return false;
            }
        }

        iteration++;
    }

    RCLCPP_ERROR(logger, "Failed to limit Cartesian speed within the maximum allowed after %d iterations.", max_iterations);
    return false;
}

// Function to perform trajectory execution
bool executeTrajectory(
    const moveit_cpp::MoveItCppPtr &moveit_cpp_ptr,
    const robot_trajectory::RobotTrajectoryPtr &trajectory,
    const std::string &planning_group,
    const rclcpp::Logger &logger)
{
    bool execution_success = moveit_cpp_ptr->execute(planning_group, trajectory);
    if (execution_success)
    {
        auto tem = moveit_cpp_ptr->getTrajectoryExecutionManager();
        bool wait_success = tem->waitForExecution();

        if (!wait_success || tem->getLastExecutionStatus() != moveit_controller_manager::ExecutionStatus::SUCCEEDED)
        {
            RCLCPP_WARN(logger, "Execution failed to complete successfully.");
            execution_success = false;
        }
    }

    if (!execution_success)
    {
        RCLCPP_ERROR(logger, "Execution failed after retries.");
    }

    return execution_success;
}

bool moveToPoseTarget(
    const moveit_cpp::MoveItCppPtr &moveit_cpp_ptr,
    const moveit_cpp::PlanningComponentPtr &planning_components,
    const geometry_msgs::msg::Pose &target_pose,
    const MovementConfig &config,
    const std::string &BASE_FRAME,
    const std::string &TCP_FRAME,
    const rclcpp::Logger &logger)
{
    // Create a PoseStamped with the target pose
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose = target_pose;
    pose_stamped.header.frame_id = BASE_FRAME;

    int retry_count = 0;
    bool execution_success = false;
    while (retry_count < config.max_exec_retries && !execution_success)
    {
        // Specify the end-effector link name
        planning_components->setGoal(pose_stamped, TCP_FRAME);

        std::vector<std::pair<moveit_cpp::PlanningComponent::PlanSolution, double>> trajectories;
        int attempts = 0;

        while (attempts < config.plan_number_limit &&
               static_cast<int>(trajectories.size()) < config.plan_number_target)
        {
            // Set the start state to the current state
            planning_components->setStartStateToCurrentState();
            auto plan_solution = planning_components->plan();

            if (plan_solution.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
            {
                double path_length = computePathLength(*plan_solution.trajectory, TCP_FRAME);
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
        if (!applyTimeParameterization(shortest_trajectory_pair->first.trajectory, config, TCP_FRAME, logger))
        {
            return false;
        }

        // Execute the trajectory using MoveItCpp
        execution_success = executeTrajectory(moveit_cpp_ptr, shortest_trajectory_pair->first.trajectory, planning_components->getPlanningGroupName(), logger);

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
    const std::string &TCP_FRAME,
    const rclcpp::Logger &logger)
{
    int retry_count = 0;
    bool execution_success = false;
    while (retry_count < config.max_exec_retries && !execution_success)
    {
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
            // Set the start state to the current state
            planning_components->setStartStateToCurrentState();
            auto plan_solution = planning_components->plan();

            if (plan_solution.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
            {
                double path_length = computePathLength(*plan_solution.trajectory, TCP_FRAME);
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
        if (!applyTimeParameterization(shortest_trajectory_pair->first.trajectory, config, TCP_FRAME, logger))
        {
            return false;
        }

        // Execute the trajectory using MoveItCpp
        execution_success = executeTrajectory(moveit_cpp_ptr, shortest_trajectory_pair->first.trajectory, planning_components->getPlanningGroupName(), logger);

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
    const std::string &TCP_FRAME,
    const rclcpp::Logger &logger)
{
    int retry_count = 0;
    bool execution_success = false;
    while (retry_count < config.max_exec_retries && !execution_success)
    {
        planning_components->setGoal(target_name);

        std::vector<std::pair<moveit_cpp::PlanningComponent::PlanSolution, double>> trajectories;
        int attempts = 0;

        while (attempts < config.plan_number_limit &&
               static_cast<int>(trajectories.size()) < config.plan_number_target)
        {
            // Set the start state to the current state
            planning_components->setStartStateToCurrentState();
            auto plan_solution = planning_components->plan();

            if (plan_solution.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
            {
                double path_length = computePathLength(*plan_solution.trajectory, TCP_FRAME);
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
        if (!applyTimeParameterization(shortest_trajectory_pair->first.trajectory, config, TCP_FRAME, logger))
        {
            return false;
        }

        // Execute the trajectory using MoveItCpp
        execution_success = executeTrajectory(moveit_cpp_ptr, shortest_trajectory_pair->first.trajectory, planning_components->getPlanningGroupName(), logger);

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
    const std::string &TCP_FRAME,
    const rclcpp::Logger &logger,
    const double linear_success_tolerance = 0.99)
{
    int retry_count = 0;
    bool execution_success = false;
    while (retry_count < config.max_exec_retries && !execution_success)
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
            planning_components->setStartStateToCurrentState();
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

            moveit::core::MaxEEFStep max_step(config.step_size, config.step_size * 2);
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

                double path_length = computePathLength(*trajectory, TCP_FRAME);
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
        if (!applyTimeParameterization(shortest_trajectory_pair->first, config, TCP_FRAME, logger))
        {
            return false;
        }

        // Execute the trajectory using MoveItCpp
        execution_success = executeTrajectory(moveit_cpp_ptr, shortest_trajectory_pair->first, planning_components->getPlanningGroupName(), logger);

        retry_count++;
    }

    if (!execution_success)
    {
        RCLCPP_ERROR(logger, "Execution failed after retries.");
        return false;
    }

    return true;
}

// Function to find a collision object with a partial ID match
bool findCollisionObject(
    const moveit_cpp::MoveItCppPtr &moveit_cpp_ptr,
    const std::string &partial_id,
    moveit_msgs::msg::CollisionObject &found_object)
{
    // create a vector of collision objects to store the available collision objects in the scene
    std::vector<moveit_msgs::msg::CollisionObject> collision_objs;

    // request the planning scene state to get the latest state
    moveit_cpp_ptr->getPlanningSceneMonitor()->requestPlanningSceneState();
    // lock the planning scene to access extracting objects
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_ptr->getPlanningSceneMonitor());
    // get collision objects to search for the partial id
    scene->getCollisionObjectMsgs(collision_objs);

    for (const auto &obj : collision_objs)
    {
        if (obj.id.find(partial_id) != std::string::npos)
        {
            // the object is found: saving the collision object on the input foun_object and returning true
            found_object = obj;
            return true;
        }
    }

    // No matching object found
    return false;
}

int main(int argc, char **argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;

    // Create the Node
    node_options.automatically_declare_parameters_from_overrides(true);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("run_moveit_cpp", "", node_options);

    // Logger
    rclcpp::Logger logger = node->get_logger();

    // Create a ROS logger
    RCLCPP_INFO(logger, "Initialize node");

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

    MovementConfig max_move_config;
    node->get_parameter_or<double>("velocity_scaling_factor", max_move_config.velocity_scaling_factor, 0.5);
    node->get_parameter_or<double>("acceleration_scaling_factor", max_move_config.acceleration_scaling_factor, 0.5);
    node->get_parameter_or<double>("step_size", max_move_config.step_size, 0.05);
    node->get_parameter_or<double>("jump_threshold", max_move_config.jump_threshold, 0.0);
    node->get_parameter_or<double>("max_cartesian_speed", max_move_config.max_cartesian_speed, 0.5);
    node->get_parameter_or<int>("max_exec_retries", max_move_config.max_exec_retries, 5);
    node->get_parameter_or<int>("plan_number_target", max_move_config.plan_number_target, 12);
    node->get_parameter_or<int>("plan_number_limit", max_move_config.plan_number_limit, 32);
    node->get_parameter_or<std::string>("smoothing_type", max_move_config.smoothing_type, "iterative_parabolic");

    MovementConfig mid_move_config = max_move_config;
    mid_move_config.velocity_scaling_factor = max_move_config.velocity_scaling_factor / 2.0;
    mid_move_config.acceleration_scaling_factor = max_move_config.acceleration_scaling_factor / 2.0;
    mid_move_config.max_cartesian_speed = 0.2;

    MovementConfig slow_move_config = max_move_config;
    slow_move_config.velocity_scaling_factor = max_move_config.velocity_scaling_factor / 4.0;
    slow_move_config.acceleration_scaling_factor = max_move_config.acceleration_scaling_factor / 4.0;
    slow_move_config.max_cartesian_speed = 0.02;

    // Create the MoveItCpp instance
    auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(node);
    // moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();
    moveit_cpp_ptr->getPlanningSceneMonitor()->requestPlanningSceneState();

    // auto my_planning_scene = moveit_cpp_ptr->getPlanningSceneMonitor()->getPlanningScene();

    // Create the PlanningComponent
    auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(robot_type, moveit_cpp_ptr);

    RCLCPP_INFO_STREAM(logger, "Set smoothing type: " << max_move_config.smoothing_type);
    auto robot_model = moveit_cpp_ptr->getRobotModel();
    auto joint_model_group = robot_model->getJointModelGroup(planning_components->getPlanningGroupName());
    const std::vector<std::string> &joint_names = joint_model_group->getActiveJointModelNames();

    for (const auto &joint_name : joint_names)
    {
        const auto &joint_model = robot_model->getJointModel(joint_name);
        const auto &bounds = joint_model->getVariableBounds(joint_name);

        RCLCPP_INFO(logger, "Joint '%s': max_velocity=%f, max_acceleration=%f",
                    joint_name.c_str(), bounds.max_velocity_, bounds.max_acceleration_);
    }
    // rclcpp::sleep_for(std::chrono::seconds(1));

    // // Create a collision object for the robot to avoid
    // auto collision_object = [BASE_FRAME]()
    // {
    //     moveit_msgs::msg::CollisionObject collision_object;
    //     collision_object.header.frame_id = BASE_FRAME;
    //     collision_object.id = "box1";

    //     shape_msgs::msg::SolidPrimitive primitive;
    //     // Define size of box
    //     primitive.type = primitive.BOX;
    //     primitive.dimensions.resize(3);
    //     primitive.dimensions[primitive.BOX_X] = 0.5;
    //     primitive.dimensions[primitive.BOX_Y] = 0.05;
    //     primitive.dimensions[primitive.BOX_Z] = 0.3;

    //     // Define pose of box (relative to frame_id)
    //     geometry_msgs::msg::Pose box_pose;
    //     box_pose.orientation.w = 1.0;
    //     box_pose.position.x = 0.15;
    //     box_pose.position.y = 0.2;
    //     box_pose.position.z = 0.15;

    //     collision_object.primitives.push_back(primitive);
    //     collision_object.primitive_poses.push_back(box_pose);
    //     collision_object.operation = collision_object.ADD;

    //     return collision_object;
    // }();

    // {
    //     // Add object to planning scene
    //     planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_ptr->getPlanningSceneMonitor());
    //     scene->processCollisionObjectMsg(collision_object);
    // }

    // // Set a target Pose
    // auto approach_pose = []()
    // {
    //     geometry_msgs::msg::Pose msg;
    //     msg.orientation.x = 1.0;
    //     msg.orientation.y = 0.0;
    //     msg.orientation.z = 0.0;
    //     msg.orientation.w = 0.0;
    //     msg.position.x = 0.1;
    //     msg.position.y = 0.3;
    //     msg.position.z = 0.2;
    //     return msg;
    // }();

    // Search for a Specific Collision Object**
    std::string search_id = "grasp"; //
    moveit_msgs::msg::CollisionObject grasp_obj;

    if (findCollisionObject(moveit_cpp_ptr, search_id, grasp_obj))
    {
        RCLCPP_INFO_STREAM(logger, "Found Collision Object:");
        RCLCPP_INFO_STREAM(logger, "ID: " << grasp_obj.id);
        RCLCPP_INFO_STREAM(logger, "Pose - X: " << grasp_obj.pose.position.x
                                                << ", Y: " << grasp_obj.pose.position.y
                                                << ", Z: " << grasp_obj.pose.position.z);
        RCLCPP_INFO_STREAM(logger, "Orientation - X: " << grasp_obj.pose.orientation.x
                                                       << ", Y: " << grasp_obj.pose.orientation.y
                                                       << ", Z: " << grasp_obj.pose.orientation.z
                                                       << ", W: " << grasp_obj.pose.orientation.w);

        // Correcting pose for testing purposes: I will try to grasp the object always from a vertical orientation
        // TODO: will have to select an axis of the object to allign with, or an axis to be perpendicular with, and generate rotations to give more grasping poses
        grasp_obj.pose.orientation.x = 1.0;
        grasp_obj.pose.orientation.y = 0.0;
        grasp_obj.pose.orientation.z = 0.0;
        grasp_obj.pose.orientation.w = 0.0;
    }
    else
    {
        RCLCPP_WARN(logger, "No collision object found with partial ID: '%s'", search_id.c_str());

        // create a pose anyway for testing purposes
        grasp_obj.pose.orientation.x = 1.0;
        grasp_obj.pose.orientation.y = 0.0;
        grasp_obj.pose.orientation.z = 0.0;
        grasp_obj.pose.orientation.w = 0.0;
        grasp_obj.pose.position.x = 0.1;
        grasp_obj.pose.position.y = 0.3;
        grasp_obj.pose.position.z = 0.2;
    }

    // create the pose messages for picking graspable object found
    geometry_msgs::msg::Pose grasp_pose = grasp_obj.pose;
    geometry_msgs::msg::Pose approach_pose = grasp_pose;

    // given the gripper shape, I need to keep the gripper 7mm higher to
    grasp_pose.position.z += 0.007;
    // I start 5 cm higher for the approach movement
    approach_pose.position.z += 0.05;

    // Define waypoints for Cartesian path
    std::vector<geometry_msgs::msg::Pose> grasp_waypoints;
    std::vector<geometry_msgs::msg::Pose> approach_waypoints;

    // Target pose (straight-line path)
    grasp_waypoints.push_back(grasp_pose);
    approach_waypoints.push_back(approach_pose);

    // Joint target
    std::vector<double> ready_joint_values = {0.0, 0.0, 1.57, 0.0, 0.0, 0.0};

    // MOVEMENTS SEQUENCE
    bool result = false;
    int counter = 0;

    // Move to joint target
    do
    {
        result = moveToJointTarget(moveit_cpp_ptr, planning_components, ready_joint_values, max_move_config, TCP_FRAME, logger);
        counter++;
    } while ((!result) && (counter < 16));

    // Move to pose target
    do
    {
        result = moveToPoseTarget(moveit_cpp_ptr, planning_components, approach_pose, mid_move_config, BASE_FRAME, TCP_FRAME, logger);
        counter++;
    } while ((!result) && (counter < 16));

    // Move to Cartesian path (grasp)
    do
    {
        result = moveCartesianPath(moveit_cpp_ptr, planning_components, grasp_waypoints, slow_move_config, TCP_FRAME, logger);
        counter++;
    } while ((!result) && (counter < 16));

    // Retract to Cartesian path (approach)
    do
    {
        result = moveCartesianPath(moveit_cpp_ptr, planning_components, approach_waypoints, mid_move_config, TCP_FRAME, logger);
        counter++;
    } while ((!result) && (counter < 16));

    // Move to named target ("home")
    do
    {
        result = moveToNamedTarget(moveit_cpp_ptr, planning_components, "home", max_move_config, TCP_FRAME, logger);
        counter++;
    } while ((!result) && (counter < 16));

    // Reset shared pointers
    planning_components.reset();
    moveit_cpp_ptr.reset();

    // Shutdown ROS
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
