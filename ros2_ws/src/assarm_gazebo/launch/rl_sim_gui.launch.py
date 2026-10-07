import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_assarm_description = get_package_share_directory('assarm_description')
    pkg_assarm_gazebo = get_package_share_directory('assarm_gazebo')

    xacro_file = os.path.join(pkg_assarm_description, 'urdf', 'assarm.xacro')
    world_file = os.path.join(pkg_assarm_gazebo, 'worlds', 'pendulum_grasp.sdf')
    bridge_config = os.path.join(pkg_assarm_gazebo, 'config', 'bridges.yaml')
    controllers_config = os.path.join(pkg_assarm_description, 'config', 'assarm_controllers_rl.yaml')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Gazebo Sim with GUI (-g)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_description_config,
            '-name', 'assarm',
            '-allow_renaming', 'true',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ]
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--config-file', bridge_config],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_position_controller', '--param-file', controllers_config],
        output='screen'
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_position_controller', '--param-file', controllers_config],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gz_sim,
        spawn_entity,
        gz_bridge,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner
    ])
