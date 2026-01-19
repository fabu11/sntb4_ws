from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    '''
    This launch file will spawn the stretch in based on the community repo:
    https://github.com/CardiffUniversityComputationalRobotics/stretch_ros2/tree/humble/stretch_gz_sim
        * main difference is the ign-based urdf to match the tb4
    '''
    
    # must source stretch_gz_sim
    pkg_stretch = get_package_share_directory('stretch_gz_sim')
    
    # stretch args 
    namespace_arg = DeclareLaunchArgument('namespace', default_value='stretch')
    x_arg = DeclareLaunchArgument('x', default_value='3.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.1')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')
    
    # set ns 
    stretch_ns = LaunchConfiguration('namespace')
    
    # robot descr
    xacro_file = os.path.join(
        pkg_stretch, "urdf", "stretch_re1", "stretch_description_standard.xacro"
    )
    robot_desc = xacro.process_file(xacro_file).toxml()  # pyright: ignore
    robot_description = {"robot_description": robot_desc}
    
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=stretch_ns,
        parameters=[robot_description, {"use_sim_time": True}],
    )
    
    sdf_file = os.path.join(
        pkg_stretch, "urdf", "stretch_re1", "stretch_re1.sdf"
    )
    
    # spawn node
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", stretch_ns, 
            "-file", sdf_file,
            "-x", LaunchConfiguration('x'),
            "-y", LaunchConfiguration('y'),
            "-z", LaunchConfiguration('z'),
            "-Y", LaunchConfiguration('yaw'),
        ]
    )
    
    # sim bridge 
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=stretch_ns,
        parameters=[{"use_sim_time": True}],
        arguments=[
            # Command topics (global in Gazebo, no model prefix)
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/joint_trajectory@trajectory_msgs/msg/JointTrajectory@gz.msgs.JointTrajectory",
            
            # State/sensor topics (model-scoped in Gazebo)
            ["/model/", stretch_ns, "/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry"],
            ["/model/", stretch_ns, "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"],
            ["/world/warehouse/model/", stretch_ns, "/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model"],
            "/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            
            # Joint control topics (global in Gazebo, as defined in SDF)
            # Using ] for ROS→Gazebo direction only
            "/joint_arm_l0@std_msgs/msg/Float64]gz.msgs.Double",
            "/joint_arm_l1@std_msgs/msg/Float64]gz.msgs.Double",
            "/joint_arm_l2@std_msgs/msg/Float64]gz.msgs.Double",
            "/joint_arm_l3@std_msgs/msg/Float64]gz.msgs.Double",
            "/joint_lift@std_msgs/msg/Float64]gz.msgs.Double",
            "/joint_wrist_yaw@std_msgs/msg/Float64]gz.msgs.Double",
            "/joint_gripper_finger_left@std_msgs/msg/Float64]gz.msgs.Double",
            "/joint_gripper_finger_right@std_msgs/msg/Float64]gz.msgs.Double",
            "/joint_head_pan@std_msgs/msg/Float64]gz.msgs.Double",
            "/joint_head_tilt@std_msgs/msg/Float64]gz.msgs.Double",
        ],
        remappings=[
            # Remap command topics to namespace
            ("/cmd_vel", "cmd_vel"),
            
            # Remap state topics to namespace
            (["/model/", stretch_ns, "/odometry"], "odom"),
            (["/world/warehouse/model/", stretch_ns, "/joint_state"], "joint_states"),
            (["/model/", stretch_ns, "/tf"], "tf"),
            
            # Remap sensor topics to namespace
            ("/lidar", "lidar"),
            ("/lidar/points", "lidar/points"),
            
            # joint control topics
            ("/joint_lift", "joint_lift"),
            ("/joint_arm_l0", "joint_arm_l0"),
            ("/joint_arm_l1", "joint_arm_l1"),
            ("/joint_arm_l2", "joint_arm_l2"),
            ("/joint_arm_l3", "joint_arm_l3"),
            ("/joint_wrist_yaw", "joint_wrist_yaw"),
            ("/joint_gripper_finger_left", "joint_gripper_finger_left"),
            ("/joint_gripper_finger_right", "joint_gripper_finger_right"),
            ("/joint_head_pan", "joint_head_pan"),
            ("/joint_head_tilt", "joint_head_tilt"),
        ],
    )
    
    return LaunchDescription([
        # handle args
        namespace_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        rsp,
        # spawn stretch in existing gz sim
        spawn,
        # enable ros2<->sim bridge
        bridge,
    ])
