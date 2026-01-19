import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    pkg_turtlebot4_ignition_bringup = get_package_share_directory('turtlebot4_ignition_bringup')
    my_pkg = get_package_share_directory('dual_robot')

    # tb4 args
    default_world_path = os.path.join(my_pkg, 'worlds', 'warehouse')
    world_arg = DeclareLaunchArgument('world', default_value=default_world_path)
    namespace_arg = DeclareLaunchArgument('namespace', default_value='tb4')
    x_arg = DeclareLaunchArgument('x', default_value='2.0')  
    y_arg = DeclareLaunchArgument('y', default_value='2.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.0')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')
    
    # stretch args - passthrough for the stretch spawn only launchfile
    stretch_namespace_arg = DeclareLaunchArgument('stretch_namespace', default_value='stretch')
    stretch_x_arg = DeclareLaunchArgument('stretch_x', default_value='5.0')
    stretch_y_arg = DeclareLaunchArgument('stretch_y', default_value='0.0')
    stretch_z_arg = DeclareLaunchArgument('stretch_z', default_value='0.1')
    stretch_yaw_arg = DeclareLaunchArgument('stretch_yaw', default_value='0.0')
    
    # launch tb4 using apt packaged bringup (hence includes sim startup)
    turtlebot4_ignition_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot4_ignition_bringup, 'launch', 'turtlebot4_ignition.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'namespace': LaunchConfiguration('namespace'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
        }.items()
    )
    
    # stretch spawn
    stretch_spawn = TimerAction(
        period=8.0,  # wait 8 seconds to clear up ign and the tb4
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(my_pkg, 'launch', 'stretch_spawn_only.launch.py')
                ),
                launch_arguments={
                    'namespace': LaunchConfiguration('stretch_namespace'),
                    'x': LaunchConfiguration('stretch_x'),
                    'y': LaunchConfiguration('stretch_y'),
                    'z': LaunchConfiguration('stretch_z'),
                    'yaw': LaunchConfiguration('stretch_yaw'),
                }.items()
            )
        ]
    )
    
    return LaunchDescription([
        # handle args
        world_arg,
        namespace_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        stretch_namespace_arg,
        stretch_x_arg,
        stretch_y_arg,
        stretch_z_arg,
        stretch_yaw_arg,
        # start sim + spawn tb4
        turtlebot4_ignition_launch,
        # spawn stretch after delay
        stretch_spawn,
    ])
