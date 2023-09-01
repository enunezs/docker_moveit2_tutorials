import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess
import xacro



# explain the following code
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
    servo_yaml = load_yaml("moveit_servo", "config/panda_simulated_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # Get URDF and SRDF
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("moveit_resources_panda_moveit_config"),
            "config",
            "panda.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_description_semantic_config = load_file(
        "moveit_resources_panda_moveit_config", "config/panda.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

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

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "panda_ros_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    # Load controllers
    load_controllers = []
    for controller in ["panda_arm_controller", "joint_state_broadcaster"]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner.py {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[robot_description],
            ),
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{"child_frame_id": "/panda_link0", "frame_id": "/world"}],
            ),
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoServer",
                name="servo_server",
                parameters=[
                    servo_params,
                    robot_description,
                    robot_description_semantic,
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::JoyToServoPub",
                name="controller_to_servo_node",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),

        ],
        output="screen",
    )
    # 
    return LaunchDescription(
        [rviz_node, ros2_control_node, container] + load_controllers
    )
    
    
    
"""

"{header:{frame_id: "panda_link0" }, 
joint_names:[panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7], 
points: [{positions:[0.32461454940247153 ,-0.46994696919150064 ,
  -0.32823108966448095 ,
  -2.3420321996129854 ,
  -0.09516602596303166 ,
  1.9612877086765625 ,
  1.0181142925718683
] }]}"

 
"
Per link control?

ros2 topic pub /panda_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{header:{frame_id: "panda_link0" }, 
joint_names: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7], 
points: [{positions:[
    0.324 ,
   -0.469 ,
   -0.328 ,
   -2.342 ,
   -0.095 ,
    1.961 ,
    1.018
],  velocities: [
  -0.00010428550838266559 ,
  0.0006120256983126306 ,
  -0.0004899240415502688 ,
  -0.0035932119354995817 ,
  -0.00015493125821821376 ,
  0.004194278256459273 ,
  -0.00020438335257771766 ]
 }]
}"


"""
