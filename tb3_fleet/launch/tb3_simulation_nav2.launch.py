import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    tb3_fleet_dir = get_package_share_directory('tb3_fleet')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    # Set TURTLEBOT3_MODEL
    os.environ['TURTLEBOT3_MODEL'] = 'burger'

    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    x_pose = LaunchConfiguration('x_pose', default='5.368')
    y_pose = LaunchConfiguration('y_pose', default='-6.654')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam', default_value='False',
        description='Whether to run SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            tb3_fleet_dir, 'maps', 'turtlebot3_world', 'map.yaml'),
        description='Full path to map yaml file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Use simulation clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(tb3_fleet_dir, 'config', 'nav2', 'nav2_params.yaml'),
        description='Full path to nav2 params file')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='True',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='True',
        description='Whether to start RViz')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            bringup_dir, 'rviz', 'nav2_default_view.rviz'),
        description='Full path to RViz config file')

    # Append GZ_SIM_RESOURCE_PATH using AppendEnvironmentVariable
    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(tb3_gazebo_dir, 'models')
    )

    set_gz_resource_path2 = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(tb3_fleet_dir, 'maps', 'turtlebot3_world', 'models')
    )
    set_gz_resource_path3 = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(os.path.expanduser('~'), '.gz', 'fuel', 'fuel.gazebosim.org', '1.0', 'openrobotics', 'models')
    )
    set_gz_resource_path4 = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(tb3_fleet_dir, 'maps', 'world_tb3', 'models')
    )

    urdf_path = os.path.join(
        tb3_gazebo_dir, 'urdf', 'turtlebot3_burger.urdf')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    # Robot state publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc,
        }],
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r -s -v2 ', os.path.join(
                tb3_fleet_dir, 'maps', 'world_tb3', 'world_tb3.world')],
            'on_exit_shutdown': 'True'
        }.items()
    )

    # Gazebo GUI client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-g -v2 ',
            'on_exit_shutdown': 'True'
        }.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, 'launch', 'spawn_turtlebot3.launch.py')),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
        }.items()
    )

    # Nav2 bringup
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'namespace':       '',
            'use_namespace':   'False',
            'slam':            slam,
            'map':             map_yaml_file,
            'use_sim_time':    use_sim_time,
            'params_file':     params_file,
            'autostart':       autostart,
            'use_composition': use_composition,
            'use_respawn':     use_respawn,
        }.items()
    )


    # RViz
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'namespace':     '',
            'use_namespace': 'False',
            'rviz_config':   rviz_config_file,
            'use_sim_time':  use_sim_time,
        }.items()
    )
    ros_gz_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ]
    )


    ld = LaunchDescription()

    ld.add_action(set_gz_resource_path)
    ld.add_action(set_gz_resource_path2)
    ld.add_action(set_gz_resource_path3)
    ld.add_action(set_gz_resource_path4)
    

    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(ros_gz_bridge_cmd)
    return ld