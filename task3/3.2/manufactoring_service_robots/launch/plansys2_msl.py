import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('manufactoring_service_robots')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': example_dir + '/pddl/domain.pddl',
          'namespace': namespace
          }.items())

    # Specify the actions
    unload_box_in_loc_cmd = Node(
        package='manufactoring_service_robots',
        executable='unload_box_in_loc_action_node',
        name='unload_box_in_loc_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    unload_box_in_ws_cmd = Node(
        package='manufactoring_service_robots',
        executable='unload_box_in_ws_action_node',
        name='unload_box_in_ws_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    enter_workstation_cmd = Node(
        package='manufactoring_service_robots',
        executable='enter_workstation_action_node',
        name='enter_workstation_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])   # Create the launch description and populate
    
    exit_workstation_cmd = Node(
        package='manufactoring_service_robots',
        executable='exit_workstation_action_node',
        name='exit_workstation_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    load_box_in_loc_cmd = Node(
        package='manufactoring_service_robots',
        executable='load_box_in_loc_action_node',
        name='load_box_in_loc_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])
        
    load_box_in_warehouse_cmd = Node(
        package='manufactoring_service_robots',
        executable='load_box_in_warehouse_action_node',
        name='load_box_in_warehouse_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    load_box_in_ws_cmd = Node(
        package='manufactoring_service_robots',
        executable='load_box_in_ws_action_node',
        name='load_box_in_ws_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    pick_box_from_loc_cmd = Node(
        package='manufactoring_service_robots',
        executable='pick_box_from_loc_action_node',
        name='pick_box_from_loc_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])
        
    pick_box_from_ws_cmd = Node(
        package='manufactoring_service_robots',
        executable='pick_box_from_ws_action_node',
        name='pick_box_from_ws_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    move_robot_cmd = Node(
        package='manufactoring_service_robots',
        executable='move_robot_action_node',
        name='move_robot_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])
        
        
    move_robot_warehouse_cmd = Node(
        package='manufactoring_service_robots',
        executable='move_robot_warehouse_action_node',
        name='move_robot_warehouse_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])
        
   
    drop_box_in_loc_cmd = Node(
        package='manufactoring_service_robots',
        executable='drop_box_in_loc_action_node',
        name='drop_box_in_loc_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])
        
        
    drop_box_in_ws_cmd = Node(
        package='manufactoring_service_robots',
        executable='drop_box_in_ws_action_node',
        name='drop_box_in_ws_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])
   
    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    ld.add_action(unload_box_in_loc_cmd)
    ld.add_action(unload_box_in_ws_cmd)
    ld.add_action(enter_workstation_cmd)
    ld.add_action(exit_workstation_cmd)
    ld.add_action(load_box_in_loc_cmd)
    ld.add_action(load_box_in_warehouse_cmd)
    ld.add_action(load_box_in_ws_cmd)
    ld.add_action(pick_box_from_loc_cmd)
    ld.add_action(pick_box_from_ws_cmd)
    ld.add_action(move_robot_cmd)
    ld.add_action(move_robot_warehouse_cmd)
    ld.add_action(drop_box_in_loc_cmd)
    ld.add_action(drop_box_in_ws_cmd)
    
    return ld
