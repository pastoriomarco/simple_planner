from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import OpaqueFunction

def launch_setup(context, *args, **kwargs):
    dof = LaunchConfiguration('dof', default=6)
    robot_type = LaunchConfiguration('robot_type', default='lite')
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='ufactory')
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    geometry_type = LaunchConfiguration('geometry_type', default='box')
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')

    moveit_configs = MoveItConfigsBuilder(
        context=context,
        controllers_name='fake_controllers',
        dof=dof,
        robot_type=robot_type,
        prefix=prefix,
        hw_ns=hw_ns,
        ros2_control_plugin='uf_robot_hardware/UFRobotFakeSystemHardware',
        add_gripper=add_gripper,
        add_bio_gripper=add_bio_gripper,
        geometry_type = geometry_type,
        geometry_mass = geometry_mass,
        geometry_height = geometry_height,
        geometry_radius = geometry_radius,
        geometry_length = geometry_length,
        geometry_width = geometry_width,
        geometry_mesh_filename = geometry_mesh_filename,
        geometry_mesh_origin_xyz = geometry_mesh_origin_xyz,
        geometry_mesh_origin_rpy = geometry_mesh_origin_rpy,
        geometry_mesh_tcp_xyz = geometry_mesh_tcp_xyz,
        geometry_mesh_tcp_rpy = geometry_mesh_tcp_rpy,
    ).to_moveit_configs()

    # moveit_configs.robot_description
    # moveit_configs.robot_description_semantic
    # moveit_configs.robot_description_kinematics
    # moveit_configs.planning_pipelines
    # moveit_configs.trajectory_execution
    # moveit_configs.planning_scene_monitor
    # moveit_configs.joint_limits
    # moveit_configs.to_dict()

    simple_planner_node = Node(
        package='hello_moveit',
        executable='simple_planner',
        output='screen',
        parameters=[
            moveit_configs.robot_description_kinematics,
            moveit_configs.joint_limits
        ],
    )
    return [simple_planner_node]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
