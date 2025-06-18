from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package="turtlesim",
             executable="turtlesim_node",
             name="sim"),
        Node(package="cps_pid_turtle",
             executable="cps_controller",
             name="controller",
             parameters=[{"shape":"square"}]),
        Node(package="cps_pid_turtle",
             executable="cps_sensor_sim",
             name="sensors",
             parameters=[{"noise_std":0.02}]),
    ])
