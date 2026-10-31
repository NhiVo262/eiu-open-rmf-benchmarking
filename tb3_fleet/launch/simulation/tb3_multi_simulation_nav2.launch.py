import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
    ExecuteProcess,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Spawn each robot on its own parking spot, read from the navigation graph the routes are
# planned on. These three are the only waypoints that start no route in any scenario, so a
# robot left here is out of every route's way. A collection runner moves the robots to the
# placement its scenario calls for before each repeat; this is only where they appear when
# the simulation first comes up.
ROBOTS = [
    {'name': 'tb3_robot1', 'x_pose': '15.092', 'y_pose': '-5.728'},   # wp_1
    {'name': 'tb3_robot2', 'x_pose': '34.238', 'y_pose': '-4.725'},   # wp_8
    {'name': 'tb3_robot3', 'x_pose': '34.268', 'y_pose': '-9.344'},   # wp_9
]


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    tb3_fleet_dir = get_package_share_directory('tb3_fleet')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    os.environ['TURTLEBOT3_MODEL'] = 'burger'

    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    use_rviz = LaunchConfiguration('use_rviz')
    use_gzclient = LaunchConfiguration('use_gzclient')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam', default_value='False',
        description='Whether to run SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            tb3_fleet_dir, 'maps', 'turtlebot3_world', 'map.yaml'),
        description='Full path to map yaml file (shared by all robots)')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Use simulation clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(tb3_fleet_dir, 'config', 'simulation', 'nav2_params.yaml'),
        description='Full path to nav2 params file (shared by all robots)')

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
        'use_rviz', default_value='False',
        description='Whether to start global RViz view (off by default to reduce CPU load during multi-robot benchmarking)')

    declare_use_gzclient_cmd = DeclareLaunchArgument(
        'use_gzclient', default_value='False',
        description='Whether to start Gazebo GUI client (off by default to reduce CPU load during multi-robot benchmarking)')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            get_package_share_directory('nav2_bringup'), 'rviz',
            'nav2_default_view.rviz'),
        description='Full path to RViz config file')

    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(tb3_gazebo_dir, 'models')
    )
    # Every furnishing world_tb3.world includes by a model:// uri is held here, one
    # directory per model name. Gazebo matches the uri against that directory name, so a
    # model missing from here stops the world loading and no robot spawns, whatever the
    # Fuel cache holds.
    set_gz_resource_path2 = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(tb3_fleet_dir, 'maps', 'turtlebot3_world', 'models')
    )
    set_gz_resource_path3 = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(tb3_fleet_dir, 'maps', 'world_tb3', 'models')
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

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-g -v2 ',
            'on_exit_shutdown': 'True'
        }.items(),
        condition=IfCondition(use_gzclient),
    )

    clock_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    urdf_path = os.path.join(
        tb3_gazebo_dir, 'urdf', 'turtlebot3_burger.urdf')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    # TB3_SIM_ROBOTS selects which of the robots above to bring up, as a comma-separated
    # list of names, and brings up all of them when unset. A baseline collection passes one
    # name, so an N=1 repeat and an N=3 repeat reach Gazebo, Nav2 and the gz bridge through
    # this same launch and differ in robot count alone.
    selected = os.environ.get('TB3_SIM_ROBOTS', '').strip()
    if selected:
        wanted = [n.strip() for n in selected.split(',') if n.strip()]
        known = {r['name'] for r in ROBOTS}
        unknown = [n for n in wanted if n not in known]
        if unknown:
            raise RuntimeError(
                'TB3_SIM_ROBOTS names no such robot: ' + ', '.join(unknown)
                + '; known names are ' + ', '.join(sorted(known)))
        robots = [r for r in ROBOTS if r['name'] in wanted]
    else:
        robots = list(ROBOTS)

    robot_groups = []
    for robot in robots:
        name = robot['name']

        model_sdf_path = os.path.join(
            tb3_fleet_dir, 'config', 'simulation', 'models', f'{name}.sdf')
        bridge_yaml_path = os.path.join(
            tb3_fleet_dir, 'config', 'simulation', 'gz_bridge', f'{name}.yaml')

        spawn_cmd = Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-name', name,
                '-file', model_sdf_path,
                '-x', robot['x_pose'],
                '-y', robot['y_pose'],
                '-z', '0.01',
            ],
        )

        bridge_cmd = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'{name}_bridge',
            output='screen',
            parameters=[{
                'config_file': bridge_yaml_path,
                'use_sim_time': use_sim_time,
            }],
        )

        robot_state_publisher_cmd = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=name,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc,
            }],

            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        )
        tf_aggregator_cmd = ExecuteProcess(
            cmd=['python3', '-m', 'tb3_fleet.tf_aggregator', '--robot-namespace', name],
            output='screen',
            condition=IfCondition(use_rviz),
        )
        initial_pose_cmd = ExecuteProcess(
            cmd=[
                'python3', '-m', 'tb3_fleet.set_initial_pose',
                '--robot-namespace', name,
                '--x', robot['x_pose'],
                '--y', robot['y_pose'],
            ],
            output='screen',
        )

        bringup_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'bringup_launch.py')),
            launch_arguments={
                'namespace':       name,
                'use_namespace':   'True',
                'slam':            slam,
                'map':             map_yaml_file,
                'use_sim_time':    use_sim_time,
                'params_file':     params_file,
                'autostart':       autostart,
                'use_composition': use_composition,
                'use_respawn':     use_respawn,
            }.items()
        )

        robot_groups.append(GroupAction([
            spawn_cmd,
            bridge_cmd,
            robot_state_publisher_cmd,
            bringup_cmd,
            tf_aggregator_cmd,
            initial_pose_cmd,
        ]))

    global_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz)
    )

    ld = LaunchDescription()

    ld.add_action(set_gz_resource_path)
    ld.add_action(set_gz_resource_path2)
    ld.add_action(set_gz_resource_path3)

    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_gzclient_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(clock_bridge_cmd)

    for group in robot_groups:
        ld.add_action(group)

    # Add the global_rviz_cmd node at the end of the LaunchDescription
    ld.add_action(global_rviz_cmd)

    return ld