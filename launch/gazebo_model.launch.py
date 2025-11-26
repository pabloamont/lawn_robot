import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import re
import xacro

def generate_launch_description():

    robotXacroName='differential_drive_robot'

    namePackage = 'lawn_robot'

    modelFileRelativePath = 'model/robot.xacro'

    worldFileRelativePath = os.path.join(get_package_share_directory('gazebo_ros'), 'worlds', 'empty.world')

    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)

    pathWorldFile = worldFileRelativePath

    # Generate URDF string from Xacro and strip the auto-generated header comment
    robotDescriptionRaw = xacro.process_file(pathModelFile).toxml()
    robotDescription = re.sub(r'<!--.*?-->', '', robotDescriptionRaw, flags=re.DOTALL)

    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch','gazebo.launch.py'))

    gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'world': pathWorldFile}.items())

    # Spawn the robot slightly above the ground so wheels, not the chassis, settle on contact
    spawnModelNode = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', robotXacroName,
                   '-x', '0', '-y', '0', '-z', '0.175'],
        output='screen'
    )

    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription,
                     'use_sim_time': True}]
    )

    jointStateBroadcasterSpawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
        output='screen'
    )

    diffDriveSpawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
        output='screen'
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(gazeboLaunch)

    launchDescriptionObject.add_action(nodeRobotStatePublisher)

    launchDescriptionObject.add_action(spawnModelNode)

    # Start controllers once the robot entity is spawned to avoid controller-manager timing issues
    launchDescriptionObject.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawnModelNode,
                on_exit=[jointStateBroadcasterSpawner, diffDriveSpawner]
            )
        )
    )

    return launchDescriptionObject
