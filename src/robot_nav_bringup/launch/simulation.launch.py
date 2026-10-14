from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    world = PathJoinSubstitution([
        FindPackageShare('robot_nav_bringup'),
        'worlds',
        'office.world.sdf',
    ])

    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('robot_nav_description'),
            'urdf',
            'robot.urdf.xacro',
        ]),
        ' use_gazebo:=true',
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py',
            ])
        ]),
        launch_arguments={'world': world}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description,
        }],
        output='screen',
    )
 

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'nav_robot'],
        output='screen',
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('robot_nav_bringup'),
                'config',
                'ekf.yaml',
            ]),
            {'use_sim_time': True},
        ],
        output='screen',
    )

    # AMCL 的初始位姿（见 config/nav2_params.yaml 里的 set_initial_pose）。
    # nav2_params.yaml 里写死的 initial_pose=(0,0,0) 只在"nav2 启动那一刻机器人正好
    # 停在出生点"时成立；仿真一直开着、nav2 反复重启时就不成立了。这个节点跟着仿真
    # 常驻，每次 AMCL 进 ACTIVE 就用当时的 /wheel/odom 真值给它播种，所以 nav2 的启动
    # 命令不用改，重启几次就重新播几次。
    amcl_seed_pose = Node(
        package='robot_nav_bringup',
        executable='amcl_seed_pose.py',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        ekf,
        amcl_seed_pose,
    ])