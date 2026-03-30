from launch import LaunchDescription
from launch_ros.actions import Node

launch_args = [
    # DeclareLaunchArgument("control", default_value=TextSubstitution(text="mpc")),
    # DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
]


def generate_launch_description():
    # opfunc = OpaqueFunction(function=launch_setup)

    # joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')

    # sim_node = Node(
    #         package='gyrobro_sim',
    #         executable='pybullet',
    #         name='sim',
    #         parameters=[config]
    #     )
    # radiolink_node = Node(
    #     package="gyrobro_radiolink",
    #     executable="gyrobro_radiolink",
    #     name="radiolink",
    #     output="screen",
    #     parameters=[config],
    # )

    robot_mode_node = Node(
        package="robot_mode_controller",
        executable="robot_mode_controller",
        name="robot_mode_controller",
        output="screen",
    )

    radiolink_node = Node(
        package="radiolink_teleop",
        executable="radiolink_teleop",
        name="radiolink_teleop",
        output="screen",
        parameters=[],
    )

    exp1_node = Node(
        package="mors_experiments",
        executable="exp1",
        name="exp1",
        output="screen",
        parameters=[],
    )

    exp2_node = Node(
        package="mors_experiments",
        executable="exp2",
        name="exp2",
        output="screen",
        parameters=[],
    )

    exp3_node = Node(
        package="mors_experiments",
        executable="exp3",
        name="exp3",
        output="screen",
        parameters=[],
    )

    exp4_node = Node(
        package="mors_experiments",
        executable="exp4",
        name="exp4",
        output="screen",
        parameters=[],
    )

    exp5_node = Node(
        package="mors_experiments",
        executable="exp5",
        name="exp5",
        output="screen",
        parameters=[],
    )

    exp6_node = Node(
        package="mors_experiments",
        executable="exp6",
        name="exp6",
        output="screen",
        parameters=[],
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[
            {
                "dev": "/dev/input/event3",
                "deadzone": 0.0,
                "autorepeat_rate": 0.0,
            }
        ],
    )

    # control_node = Node(
    #     package="composition-test",
    #     executable="compositor",
    #     name="compositor",
    #     output="screen",
    #     parameters=[config],
    # )

    ld = LaunchDescription(launch_args)
    ld.add_action(joy_node)
    # ld.add_action(radiolink_node)
    ld.add_action(robot_mode_node)

    # ld.add_action(exp1_node)
    # ld.add_action(exp2_node)
    # ld.add_action(exp3_node)
    # ld.add_action(exp4_node)
    # ld.add_action(exp5_node)
    # ld.add_action(exp6_node)

    # ld.add_action(opfunc)
    # ld.add_action(control_node)

    return ld
