from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package="turtlesim",
             executable="turtlesim_node",
             name="sim"),
        # stubs – will be implemented in Parts B and C
        Node(package="cps_pid_turtle",
             executable="p_controller",
             name="controller",
             parameters=[{"shape":"triangle","side":2.0}],
             output="screen"),
     #    Node(package="cps_pid_turtle",
     #         executable="sensor_sim",
     #         name="sensors",
     #         parameters=[{"noise_std":0.02}],
     #         output="screen"),
    ])
