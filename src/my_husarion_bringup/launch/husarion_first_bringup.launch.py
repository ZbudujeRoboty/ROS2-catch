from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    turtlebot_spawner_node = Node(
        package="husarion_mini_project",
        executable="turtlebot3_spawner",
        parameters=[
            {"spawn_period": 15.0},
            {"prefix": "ROBOT_"}
        ]
    )

    turtlebot_controller_node = Node(
        package="husarion_mini_project",
        executable="turtlebot3_controller",
        parameters=[
            {"closest": True}
        ]
    )

    ld.add_action(turtlebot_spawner_node)
    ld.add_action(turtlebot_controller_node)
    return ld