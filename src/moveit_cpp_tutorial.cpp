#include <rclcpp/rclcpp.hpp>
#include <memory>

// MoveItCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

// Cartesian Interpolator
#include <moveit/robot_state/cartesian_interpolator.h>

// Trajectory Processing
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

// Planning Scene
#include <moveit/planning_scene/planning_scene.h>

#include <geometry_msgs/msg/point_stamped.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace rvt = rviz_visual_tools;

// Logger
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp_tutorial");

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    RCLCPP_INFO(LOGGER, "Initialize node");

    node_options.automatically_declare_parameters_from_overrides(true);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("run_moveit_cpp", "", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]()
                { executor.spin(); })
        .detach();

    static const std::string PLANNING_GROUP = "lite6";
    static const std::string BASE_FRAME = "link_base";
    static const std::string TCP_FRAME = "link_tcp";

    /* Otherwise robot with zeros joint_states */
    rclcpp::sleep_for(std::chrono::seconds(1));

    RCLCPP_INFO(LOGGER, "Starting MoveIt Tutorials...");

    auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(node);
    moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();

    auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
    auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
    auto robot_start_state = planning_components->getStartState();
    auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);

    // Visualization
    moveit_visual_tools::MoveItVisualTools visual_tools(node, BASE_FRAME, "moveit_cpp_tutorial",
                                                        moveit_cpp_ptr->getPlanningSceneMonitor());
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 0.75;
    visual_tools.publishText(text_pose, "MoveItCpp_Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    // Start the demo
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    // Planning with MoveItCpp
    // ^^^^^^^^^^^^^^^^^^^^^^^
    // There are multiple ways to set the start and the goal states of the plan
    // they are illustrated in the following plan examples
    //
    // Plan #1
    // ^^^^^^^
    //
    // We can set the start state of the plan to the current state of the robot
    planning_components->setStartStateToCurrentState();

    // The first way to set the goal of the plan is by using geometry_msgs::PoseStamped ROS message type as follow
    geometry_msgs::msg::PoseStamped target_pose1;
    target_pose1.header.frame_id = BASE_FRAME;
    target_pose1.pose.orientation.x = 1.0;
    target_pose1.pose.orientation.y = 0.0;
    target_pose1.pose.orientation.z = 0.0;
    target_pose1.pose.orientation.w = 0.0;
    target_pose1.pose.position.x = 0.20;
    target_pose1.pose.position.y = -0.1;
    target_pose1.pose.position.z = 0.15;
    planning_components->setGoal(target_pose1, TCP_FRAME);

    // Now, we call the PlanningComponents to compute the plan and visualize it.
    // Note that we are just planning
    auto plan_solution1 = planning_components->plan();

    // Check if PlanningComponents succeeded in finding the plan
    if (plan_solution1)
    {
        // Visualize the start pose in Rviz
        visual_tools.publishAxisLabeled(robot_start_state->getGlobalLinkTransform(TCP_FRAME), "start_pose");
        // Visualize the goal pose in Rviz
        visual_tools.publishAxisLabeled(target_pose1.pose, "target_pose");
        visual_tools.publishText(text_pose, "setStartStateToCurrentState", rvt::WHITE, rvt::XLARGE);
        // Visualize the trajectory in Rviz
        visual_tools.publishTrajectoryLine(plan_solution1.trajectory, joint_model_group_ptr);
        visual_tools.trigger();

        /* Uncomment if you want to execute the plan */
        planning_components->execute(); // Execute the plan */
    }

    // Plan #1 visualization:
    //
    // .. image:: images/moveitcpp_plan1.png
    //    :width: 250pt
    //    :align: center
    //
    // Start the next plan
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();

    // Plan #2
    // ^^^^^^^
    //
    // Here we will set the current state of the plan using
    // moveit::core::RobotState
    auto start_state = *(moveit_cpp_ptr->getCurrentState());
    geometry_msgs::msg::Pose start_pose;
    start_pose.orientation.x = 1.0;
    start_pose.orientation.y = 0.0;
    start_pose.orientation.z = 0.0;
    start_pose.orientation.w = 0.0;
    start_pose.position.x = 0.20;
    start_pose.position.y = -0.1;
    start_pose.position.z = 0.15;

    start_state.setFromIK(joint_model_group_ptr, start_pose);

    planning_components->setStartState(start_state);

    // Having executed plan #1, we create a new target with a little difference on Z:
    geometry_msgs::msg::PoseStamped target_poseE;
    target_poseE.header.frame_id = BASE_FRAME;
    target_poseE.pose.orientation.x = 1.0;
    target_poseE.pose.orientation.y = 0.0;
    target_poseE.pose.orientation.z = 0.0;
    target_poseE.pose.orientation.w = 0.0;
    target_poseE.pose.position.x = 0.20;
    target_poseE.pose.position.y = -0.1;
    target_poseE.pose.position.z = 0.05;
    planning_components->setGoal(target_poseE, TCP_FRAME);

    // And plan to it:
    auto plan_solution2 = planning_components->plan();
    if (plan_solution2)
    {
        moveit::core::RobotState robot_state(robot_model_ptr);
        moveit::core::robotStateMsgToRobotState(plan_solution2.start_state, robot_state);

        visual_tools.publishAxisLabeled(robot_state.getGlobalLinkTransform(TCP_FRAME), "start_pose");
        visual_tools.publishAxisLabeled(target_pose1.pose, "target_pose");
        visual_tools.publishText(text_pose, "moveit::core::RobotState_Start_State", rvt::WHITE, rvt::XLARGE);
        visual_tools.publishTrajectoryLine(plan_solution2.trajectory, joint_model_group_ptr);
        visual_tools.trigger();

        /* Uncomment if you want to execute the plan */
        planning_components->execute(); // Execute the plan */
    }

    // Plan #2 visualization:
    //
    // .. image:: images/moveitcpp_plan2.png
    //    :width: 250pt
    //    :align: center
    //
    // Start the next plan
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();

    // Plan #3
    // ^^^^^^^
    //
    // We can also set the goal of the plan using
    // moveit::core::RobotState
    auto target_state = *robot_start_state;
    geometry_msgs::msg::Pose target_pose2;
    target_pose2.orientation.x = 1.0;
    target_pose2.orientation.y = 0.0;
    target_pose2.orientation.z = 0.0;
    target_pose2.orientation.w = 0.0;
    target_pose2.position.x = 0.25;
    target_pose2.position.y = -0.05;
    target_pose2.position.z = 0.3;

    target_state.setFromIK(joint_model_group_ptr, target_pose2);

    planning_components->setStartStateToCurrentState();
    planning_components->setGoal(target_state);

    // We will reuse the old start that we had and plan from it.
    auto plan_solution3 = planning_components->plan();
    if (plan_solution3)
    {
        moveit::core::RobotState robot_state(robot_model_ptr);
        moveit::core::robotStateMsgToRobotState(plan_solution3.start_state, robot_state);

        visual_tools.publishAxisLabeled(robot_state.getGlobalLinkTransform(TCP_FRAME), "start_pose");
        visual_tools.publishAxisLabeled(target_pose2, "target_pose");
        visual_tools.publishText(text_pose, "moveit::core::RobotState_Goal_Pose", rvt::WHITE, rvt::XLARGE);
        visual_tools.publishTrajectoryLine(plan_solution3.trajectory, joint_model_group_ptr);
        visual_tools.trigger();

        /* Uncomment if you want to execute the plan */
        planning_components->execute(); // Execute the plan */
    }

    // Plan #3 visualization:
    //
    // .. image:: images/moveitcpp_plan3.png
    //    :width: 250pt
    //    :align: center
    //
    // Start the next plan
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();

    // Plan #4
    // ^^^^^^^
    //
    // We can set the start state of the plan to the current state of the robot
    // We can set the goal of the plan using the name of a group states
    // for panda robot we have one named robot state for "panda_arm" planning group called "ready"
    // see `panda_arm.xacro
    // <https://github.com/ros-planning/moveit_resources/blob/ros2/panda_moveit_config/config/panda_arm.xacro#L13>`_

    /* // Set the start state of the plan from a named robot state */
    /* planning_components->setStartState("ready"); // Not implemented yet */
    // Set the goal state of the plan from a named robot state
    planning_components->setGoal("home");

    // Again we will reuse the old start that we had and plan from it.
    auto plan_solution4 = planning_components->plan();
    if (plan_solution4)
    {
        moveit::core::RobotState robot_state(robot_model_ptr);
        moveit::core::robotStateMsgToRobotState(plan_solution4.start_state, robot_state);

        visual_tools.publishAxisLabeled(robot_state.getGlobalLinkTransform(TCP_FRAME), "start_pose");
        visual_tools.publishAxisLabeled(robot_start_state->getGlobalLinkTransform(TCP_FRAME), "target_pose");
        visual_tools.publishText(text_pose, "Goal_Pose_From_Named_State", rvt::WHITE, rvt::XLARGE);
        visual_tools.publishTrajectoryLine(plan_solution4.trajectory, joint_model_group_ptr);
        visual_tools.trigger();

        /* Uncomment if you want to execute the plan */
        planning_components->execute(); // Execute the plan */
    }

    // Plan #4 visualization:
    //
    // .. image:: images/moveitcpp_plan4.png
    //    :width: 250pt
    //    :align: center
    //
    // Start the next plan
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();

    // Plan #5
    // ^^^^^^^
    //
    // We can also generate motion plans around objects in the collision scene.
    //
    // First we create the collision object
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = BASE_FRAME;
    collision_object.id = "box";

    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {0.05, 0.4, 0.1};

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 0.25;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.1;

    collision_object.primitives.push_back(box);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    // Add object to planning scene
    { // Lock PlanningScene
        planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_ptr->getPlanningSceneMonitor());
        scene->processCollisionObjectMsg(collision_object);
    } // Unlock PlanningScene
    planning_components->setStartStateToCurrentState();
    // Modified to previous target for lite6
    planning_components->setGoal(target_state);

    auto plan_solution5 = planning_components->plan();
    if (plan_solution5)
    {
        visual_tools.publishText(text_pose, "Planning_Around_Collision_Object", rvt::WHITE, rvt::XLARGE);
        visual_tools.publishTrajectoryLine(plan_solution5.trajectory, joint_model_group_ptr);
        visual_tools.trigger();

        /* Uncomment if you want to execute the plan */
        planning_components->execute(); // Execute the plan */
    }

    // Plan #5 visualization:
    //
    // .. image:: images/moveitcpp_plan5.png
    //    :width: 250pt
    //    :align: center
    //
    // END_TUTORIAL
    visual_tools.prompt("Press 'next' to end the demo");
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();


    // Plan #6
    // ^^^^^^^
    // Added linear motion 

    planning_components->setStartStateToCurrentState();

    // Get the current robot state
    auto current_state = moveit_cpp_ptr->getCurrentState();

    // Define the target pose (move along the Z-axis)
    Eigen::Isometry3d target_linear = current_state->getGlobalLinkTransform(TCP_FRAME);
    target_linear.translation().z() -= 0.1; // Move down 10 cm

    // Parameters for Cartesian interpolation
    double max_step_translation = 0.01; // Maximum step size in Cartesian space
    double max_step_rotation = 0.0;     // No rotation
    moveit::core::MaxEEFStep max_step(max_step_translation, max_step_rotation);

    moveit::core::JumpThreshold jump_threshold;
    jump_threshold.revolute = 0.0;
    jump_threshold.prismatic = 0.0; // Disable jump detection

    std::vector<moveit::core::RobotStatePtr> trajectory_states;

    // Get the link model for the TCP frame
    const moveit::core::LinkModel *link_model = robot_model_ptr->getLinkModel(TCP_FRAME);

    // Define the validity callback
    auto planning_scene = moveit_cpp_ptr->getPlanningSceneMonitor()->getPlanningScene();
    moveit::core::GroupStateValidityCallbackFn validity_callback =
        [planning_scene](moveit::core::RobotState *state, const moveit::core::JointModelGroup *group, const double *joint_group_variable_values)
    {
        state->setJointGroupPositions(group, joint_group_variable_values);
        state->update();
        return !planning_scene->isStateColliding(*state, group->getName());
    };

    // Compute the Cartesian path
    double achieved_fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
        current_state.get(), joint_model_group_ptr, trajectory_states, link_model,
        target_linear, true /* global_reference_frame */, max_step, jump_threshold,
        validity_callback);

    if (achieved_fraction > 0.99)
    {
        // Create a RobotTrajectory as a shared pointer
        auto trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_ptr, PLANNING_GROUP);

        // Add the waypoints to the trajectory
        for (size_t i = 0; i < trajectory_states.size(); ++i)
        {
            trajectory->addSuffixWayPoint(trajectory_states[i], 0.0); // Time is 0.0, will be updated
        }

        // Time parameterization
        trajectory_processing::IterativeParabolicTimeParameterization time_param;
        bool time_param_success = time_param.computeTimeStamps(*trajectory);
        if (!time_param_success)
        {
            RCLCPP_ERROR(LOGGER, "Failed to compute time stamps for the trajectory");
        }

        // Visualize the trajectory
        visual_tools.publishTrajectoryLine(*trajectory, joint_model_group_ptr);
        visual_tools.trigger();

        // Execute the trajectory using MoveItCpp
        bool execution_success = moveit_cpp_ptr->execute(PLANNING_GROUP, trajectory);
        if (!execution_success)
        {
            RCLCPP_ERROR(LOGGER, "Failed to execute the trajectory");
        }
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Failed to compute Cartesian path");
    }

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to end the demo");
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();

    RCLCPP_INFO(LOGGER, "Shutting down.");
    rclcpp::shutdown();
    return 0;
}
