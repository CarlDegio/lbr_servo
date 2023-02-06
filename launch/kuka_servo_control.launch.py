import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
import xacro


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # Get parameters for the Servo node
    servo_yaml = load_yaml("lbr_servo", "config/kuka_joint_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # Get URDF and SRDF
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("lbr_servo"),
            "urdf",
            "iiwa14",
            "iiwa14.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_description_semantic_config = load_file(
        "lbr_servo", "urdf/iiwa14/iiwa14.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # The servo cpp interface demo
    # Creates the Servo node and publishes commands to it
    servo_node = Node(
        package="lbr_servo",
        executable="servo_command",
        output="screen",
        parameters=[servo_params, robot_description, robot_description_semantic],
    )

    # Publishes tf's for the robot
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # RViz
    rviz_config_file = (
            get_package_share_directory("moveit2_tutorials")
            + "/config/demo_rviz_config.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[robot_description, robot_description_semantic],
    )


    return LaunchDescription(
        [servo_node, robot_state_publisher, rviz_node]
    )