import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction,
                            LogInfo, RegisterEventHandler, TimerAction, IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import subprocess
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
import xacro
import yaml

# def stop_tbot(_launch_context):
#     subprocess.run('ros2 topic pub -t 1 /turtle/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"', shell=True)

# TODO: use ros2 service call /reset_simulation std_srvs/srv/Empty to bringup world separately, next year.

def generate_launch_description():

    # pkg_sjtu_drone_bringup = get_package_share_directory('sjtu_drone_bringup')
    # pkg_sjtu_drone_description = get_package_share_directory('sjtu_drone_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_ee4308_bringup = get_package_share_directory('ee4308_bringup')
    # pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    default_turtle_x = os.getenv('EE4308_TURTLE_X', '-2.0')
    default_turtle_y = os.getenv('EE4308_TURTLE_Y', '-0.5')
    default_drone_x = os.getenv('EE4308_DRONE_X', '0.0')
    default_drone_y = os.getenv('EE4308_DRONE_Y', '-4.0')
    default_drone_z = os.getenv('EE4308_DRONE_Z', '0.0')
    default_world = os.getenv('EE4308_WORLD', 'turtlebot3_world.world') # TODO: to provide coordinates in world sh, next year.
    default_world = os.path.join(pkg_ee4308_bringup, 'worlds', default_world)
    default_use_sim_time = 'true'

    ee4308_task = "proj2"
    tbot_model = "burger"
    turtle_ns = "turtle"
    drone_ns = "drone" # overwritten nelow
    drone_yaml = os.path.join(pkg_ee4308_bringup, 'params', 'drone.yaml')
    # get drone nss
    with open(drone_yaml, 'r') as f:
        yaml_dict = yaml.load(f, Loader=yaml.FullLoader)
        drone_ns = yaml_dict["namespace"]

    turtle_x = LaunchConfiguration('turtle_x')
    turtle_x_arg = DeclareLaunchArgument('turtle_x', default_value=default_turtle_x)

    turtle_y = LaunchConfiguration('turtle_y')
    turtle_y_arg = DeclareLaunchArgument('turtle_y', default_value=default_turtle_y)

    drone_x = LaunchConfiguration('drone_x')
    drone_x_arg = DeclareLaunchArgument('drone_x', default_value=default_drone_x)

    drone_y = LaunchConfiguration('drone_y')
    drone_y_arg = DeclareLaunchArgument('drone_y', default_value=default_drone_y)

    drone_z = LaunchConfiguration('drone_z')
    drone_z_arg = DeclareLaunchArgument('drone_z', default_value=default_drone_z)
    
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument('world', default_value=default_world)
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=default_use_sim_time)
    
    # get drone description
    
    drone_description_config = xacro.process_file(
        os.path.join(pkg_ee4308_bringup, "urdf", "sjtu_drone.urdf.xacro"), 
        mappings={"params_path": drone_yaml})
    drone_desc = drone_description_config.toxml()

    # get turtle description
    turtle_description_config = xacro.process_file(os.path.join(pkg_ee4308_bringup, 'urdf', 'turtlebot3_burger.urdf.xacro'))
    turtle_desc = turtle_description_config.toxml()

    # get proj2.yaml
    params_yaml = os.path.join(pkg_ee4308_bringup, "params", ee4308_task + ".yaml")


    # open gazebo server
    launch_gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world,
                          'verbose': "true",
                        #   'extra_gazebo_args': 'verbose'
                          }.items()
    )

    # open gazebo client.
    launch_gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        # launch_arguments={'verbose': 'true'}.items()
    )

    # drone state publisher
    node_drone_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=drone_ns,
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "robot_description": drone_desc, "frame_prefix": drone_ns + "/"}],
        arguments=[drone_desc]
    )

    # drone joint publisher
    node_drone_joint_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=drone_ns,
        output='screen',
    )

    # turtle state publisher
    # wheel_left_joint and wheel_right_joint tfs (and parent link base_footprint) are not published correctly by the gazebo_ros diff drive plugin.
    node_turtle_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=turtle_ns,
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "robot_description": turtle_desc, "frame_prefix": turtle_ns + "/"}],
        arguments=[turtle_desc]
    )

    # # turtle joint publisher # use the gazebo_ros_joint_publisher specified in turtle urdf.
    # node_turtle_joint_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     namespace=turtle_ns,
    #     output='screen',
    # )

    # static transform for drone
    node_drone_static_tf = Node(
        package="tf2_ros",
        name="drone_static_tf",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", f"{drone_ns}/odom"],
        output="screen"
    )

    # static transform for turtle
    node_turtle_static_tf = Node(
        package="tf2_ros",
        name="turtle_static_tf",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", f"{turtle_ns}/odom"],
        output="screen"
    )

    node_turtle_mapper = Node(
        namespace=turtle_ns,
        package='ee4308_turtle',
        executable='mapper',
        parameters=[params_yaml],
        output='screen',
        emulate_tty=True,
    )

    node_turtle_planner = Node(
        namespace=turtle_ns,
        package='ee4308_turtle',
        executable='planner_smoother',
        parameters=[params_yaml],
        output='screen',
        emulate_tty=True,
    )
    
    node_turtle_behavior = Node(
        namespace=turtle_ns,
        package='ee4308_turtle',
        executable='behavior',
        parameters=[params_yaml],
        output='screen',
        emulate_tty=True,
    )

    node_turtle_controller = Node(
        namespace=turtle_ns,
        package='ee4308_turtle',
        executable='controller',
        parameters=[params_yaml],
        output='screen',
        emulate_tty=True,
    )


    node_drone_behavior = Node(
        namespace=drone_ns,
        package='ee4308_drone',
        executable='behavior',
        parameters=[params_yaml],
        arguments=[drone_x, drone_y, drone_z],
        output='screen',
        emulate_tty=True,
    )

    node_drone_controller = Node(
        namespace=drone_ns,
        package='ee4308_drone',
        name='controller2',
        executable='controller',
        parameters=[params_yaml],
        output='screen',
        emulate_tty=True,
    )

    node_drone_smoother = Node(
        namespace=drone_ns,
        package='ee4308_drone',
        executable='smoother',
        parameters=[params_yaml],
        output='screen',
        emulate_tty=True,
    )

    node_drone_estimator = Node(
        namespace=drone_ns,
        package='ee4308_drone',
        executable='estimator',
        parameters=[params_yaml],
        arguments=[drone_x, drone_y, drone_z],
        output='screen',
        emulate_tty=True,
    )

    # rviz
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_ee4308_bringup, "rviz", ee4308_task + ".rviz"),],
        output="screen"
    )
    
    # spawn turtle
    node_spawn_turtle = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', tbot_model,
            '-topic', f"{turtle_ns}/robot_description", # os.path.join(pkg_ee4308_bringup, 'models', 'turtlebot3_burger', 'model.sdf'),
            '-x', turtle_x,
            '-y', turtle_y,
            '-z', '0.01',
            "-robot_namespace", turtle_ns
        ],
        output='screen',
        on_exit=[node_rviz]
    )

    # spawn drone
    node_spawn_drone = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', drone_ns, # name of robot
            '-topic', f"{drone_ns}/robot_description",
            '-x', drone_x,
            '-y', drone_y,
            '-z', drone_z,
            "-robot_namespace", drone_ns
        ],
        output='screen',
        on_exit=[node_spawn_turtle] # spawn the drone first bcos it may collide with the turtle. (note tbot is at origin, spawner spawns everything at origin before moving it to "initial pose")
    )

    # # Create the launch description and populate
    ld = LaunchDescription()

    # # Declare the launch options
    ld.add_action(turtle_x_arg)
    ld.add_action(turtle_y_arg)
    ld.add_action(drone_x_arg)
    ld.add_action(drone_y_arg)
    ld.add_action(drone_z_arg)
    ld.add_action(world_arg)
    ld.add_action(use_sim_time_arg)

    # ld.add_action(launch_gzclient)
    ld.add_action(launch_gzserver)

    ld.add_action(node_drone_static_tf)
    ld.add_action(node_turtle_static_tf)
    ld.add_action(node_drone_state_publisher)
    ld.add_action(node_drone_joint_publisher)
    ld.add_action(node_turtle_state_publisher)
    ld.add_action(node_spawn_drone)
    # ld.add_action(event_drone_estimator)
    # ld.add_action(event_rviz)

    # ld.add_action(node_turtle_estimator)
    ld.add_action(node_turtle_mapper)
    ld.add_action(node_turtle_planner)
    ld.add_action(node_turtle_behavior)
    ld.add_action(node_turtle_controller)

    ld.add_action(node_drone_smoother)
    ld.add_action(node_drone_estimator)
    ld.add_action(node_drone_behavior)
    ld.add_action(node_drone_controller)

    return ld