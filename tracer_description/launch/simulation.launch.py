import os
import sys
import xacro

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    # Acquiring robot description XACRO file
    install_dir = get_package_prefix('tracer_description')

    # Installing Gazebo Model and Plugin Paths
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + \
            ':' + install_dir + '/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + "/share"
    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + \
            ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    pkg_tracer_description = get_package_share_directory(
        'tracer_description')

    xacro_file = os.path.join(
        pkg_tracer_description, 'models/xacro', 'tracer.xacro')
    assert os.path.exists(
        xacro_file), "The tracer.xacro doesn't exist in " + str(xacro_file)

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()
    robot_description_param = {'robot_description': robot_description}

    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch', 'gzserver.launch.py',
            ])
        ]),
        launch_arguments={
            'verbose': 'true',
            # 'world': TextSubstitution(text=str(gazebo_world))
        }.items()
    )

    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch', 'gzclient.launch.py'
            ])
        ]),
        launch_arguments={
            'verbose': 'false',
        }.items()
    )

    spawn_tracer_node = Node(
        package='tracer_description',
        executable='spawn_tracer',
        namespace='/tracer',
        arguments=[robot_description],
        output='screen',
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='/tracer',
        parameters=[robot_description_param],
    )

    # Loading Controllers
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/tracer/controller_manager"],
    )

    ld.add_action(robot_state_publisher_node)

    # Launching Gazebo
    ld.add_action(gzserver_launch)
    ld.add_action(gzclient_launch)

    # Spawning Robot
    ld.add_action(spawn_tracer_node)

    # Loading Controllers
    # ld.add_action(joint_state_broadcaster)

    return ld
