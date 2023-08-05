import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    pkg_share = FindPackageShare(package='aster_controller').find('aster_controller')
    config_path = os.path.join(pkg_share, 'config/params.yaml')

    config = LaunchConfiguration('config')
    config_cmd = DeclareLaunchArgument(
        name='config',
        default_value=config_path,
        description='Absolute path to the params file'
    )

    robot_controller_node = Node(
        package="aster_controller",
        executable="aster_controller",
        parameters=[config],
        remappings= [("/receive_serial", "/stm32/receive_serial"),
                     ("/send_serial", "/stm32/send_serial")]
    )

    robot_serial_node_stm32 = Node(
        namespace = "stm32",
        package="aster_controller",
        executable="aster_serial",
        parameters=[config],
        remappings= [("/receive_serial", "/stm32/receive_serial"),
                     ("/send_serial", "/stm32/send_serial")]
    )

    robot_odometry_node = Node(
        package="aster_controller",
        executable="aster_odometry",
        parameters=[config],
        remappings= [("/receive_serial", "/stm32/receive_serial"),
                     ("/odom", "/odometry")]
    )

    robot_imu_node = Node(
        package="aster_controller",
        executable="aster_imu",
        remappings= [("/receive_serial", "/stm32/receive_serial")]
    )

    robot_imu_filter_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter",
        output="screen",
        parameters=[config]
    )

    robot_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[config]
    )

    ld = LaunchDescription()

    ld.add_action(config_cmd)
    ld.add_action(robot_serial_node_stm32)
    ld.add_action(robot_controller_node)
    ld.add_action(robot_odometry_node)
    #ld.add_action(robot_imu_node)
    #ld.add_action(robot_imu_filter_node)
    #ld.add_action(robot_ekf_node)

    return ld
