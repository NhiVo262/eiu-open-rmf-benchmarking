import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():

    rmf_viz_sched_share = get_package_share_directory('rmf_visualization_schedule')
    tb3_fleet_share = get_package_share_directory('tb3_fleet')

    default_viz_config = os.path.join(rmf_viz_sched_share, 'config', 'rmf.rviz')
    adapter_nav_graph = os.path.join(tb3_fleet_share, 'maps', 'tb3_world', 'nav_graphs', '0.yaml')
    adapter_config_file = os.path.join(tb3_fleet_share, 'config', 'fleet', 'tb3_simulation_config.yaml')

    
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file') 
    initial_map = LaunchConfiguration('initial_map')
    headless = LaunchConfiguration('headless')
    bidding_time_window = LaunchConfiguration('bidding_time_window')
    use_unique_hex_string_with_task_id = LaunchConfiguration('use_unique_hex_string_with_task_id')
    server_uri = LaunchConfiguration('server_uri')
    viz_config_file = LaunchConfiguration('viz_config_file')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use the /clock topic for time to sync with simulation'),
        DeclareLaunchArgument('config_file', default_value=os.path.join(tb3_fleet_share, 'maps', 'tb3_world', 'tb3_world.building.yaml'), description='Building description file required by rmf_building_map_tools'),
        DeclareLaunchArgument('initial_map', default_value='tb3_world', description='Initial map name for the visualizer'),
        DeclareLaunchArgument('headless', default_value='true', description='Do not launch rviz; launch gazebo in headless mode'),
        DeclareLaunchArgument('bidding_time_window', default_value='2.0', description='Time window in seconds for task bidding process'),
        DeclareLaunchArgument('use_unique_hex_string_with_task_id', default_value='true', description='Appends a unique hex string to the task ID'),
        DeclareLaunchArgument('server_uri', default_value='', description='The URI of the api server to transmit state and task information.'),
        DeclareLaunchArgument('viz_config_file', default_value=default_viz_config, description='Path to RViz config file'),
    ]

    core_nodes = [
        Node(
            package='rmf_traffic_ros2',
            executable='rmf_traffic_schedule',
            name='rmf_traffic_schedule_primary',
            output='both',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='rmf_traffic_ros2',
            executable='rmf_traffic_blockade',
            output='both',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='rmf_building_map_tools',
            executable='building_map_server',
            arguments=[config_file], 
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='rmf_fleet_adapter',
            executable='door_supervisor',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='rmf_fleet_adapter',
            executable='lift_supervisor',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='rmf_task_ros2',
            executable='rmf_task_dispatcher',
            output='screen',
            arguments=['--ros-args', '--log-level', 'debug'],

            parameters=[{
                'use_sim_time': False,
                'bidding_time_window': bidding_time_window,
                'use_unique_hex_string_with_task_id': use_unique_hex_string_with_task_id,
                'server_uri': server_uri
            }]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', initial_map],
            output='screen'
        )
    ]

    viz_nodes = [
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', viz_config_file],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=UnlessCondition(headless)
        ),
        Node(
            package='rmf_visualization_navgraphs',
            executable='navgraph_visualizer_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'initial_map_name': initial_map
            }]
        ),
        Node(
            package='rmf_visualization_floorplans',
            executable='floorplan_visualizer_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'initial_map_name': initial_map
            }]
        ),
        Node(
            package='rmf_visualization_schedule',
            executable='schedule_visualizer_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'initial_map_name': initial_map
            }]
        ),
        Node(
            package='rmf_visualization_fleet_states',
            executable='fleetstates_visualizer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ]

    fleet_adapter_cmd = ExecuteProcess(
        cmd=[
            'python3', '-m', 'tb3_fleet.tb3_fleet_adapter',
            '-c', adapter_config_file,
            '-n', adapter_nav_graph,
            '--zenoh-config', os.path.join(tb3_fleet_share, 'config', 'zenoh', 'tb3_fleet_adapter_zenoh_config.json5'),

        ],
        output='screen',
    )

    ld = LaunchDescription()

    for arg in declare_args:
        ld.add_action(arg)

    for node in core_nodes:
        ld.add_action(node)

    for node in viz_nodes:
        ld.add_action(node)

    ld.add_action(fleet_adapter_cmd)

    return ld