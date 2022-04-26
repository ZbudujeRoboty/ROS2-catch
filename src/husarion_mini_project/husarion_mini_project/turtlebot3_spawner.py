#!/usr/bin/env python3
import rclpy
import random
import math
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.srv import DeleteEntity
from functools import partial
import os
from ament_index_python.packages import get_package_share_directory
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle

class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtlebot_spawner")

        self.declare_parameter("spawn_period", 15.0)
        self.declare_parameter("prefix", "ROBOT")
        self.spawn_period = self.get_parameter("spawn_period").value
        self.turtlebot_name_prefix_ = self.get_parameter("prefix").value

        self.turtlebot_counter_ = 0
        self.alive_turtles_ = []

        self.spawn_timer_ = self.create_timer(self.spawn_period, self.callback_timer)
        self.alive_pub_ = self.create_publisher(TurtleArray, "/alive_turtles",10)
        self.catch_server_ = self.create_service(CatchTurtle, "/catch_turtle", self.callback_catch_turtle)

        self.get_logger().info("Turtlebot Spawner has been started!")

    def callback_catch_turtle(self, request, response):
        self.call_kill_server(request.turtle_name)
        response.success = True
        return response

    def call_kill_server(self, catched_turtle_name):
        client = self.create_client(DeleteEntity, "/delete_entity")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server...")        

        request = DeleteEntity.Request()
        request.name = catched_turtle_name
        
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_kill, catched_turtle_name = catched_turtle_name))

    def callback_kill(self, future, catched_turtle_name):
        try:
            future.result()
            self.get_logger().info("Killed " + catched_turtle_name)
            for (i, turtle) in enumerate(self.alive_turtles_):
                if turtle.turtle_name == catched_turtle_name:
                    del self.alive_turtles_[i]
                    self.publish_alive_turtles()
                    break
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles_array = self.alive_turtles_
        self.alive_pub_.publish(msg)

    def callback_timer(self):
        self.turtlebot_counter_ += 1
        
        x_ = random.randint(0,801)/100.0 - 4.0# random.uniform(0.0,11.0)
        y_ = random.randint(0,801)/100.0 - 4.0
        name_ = self.turtlebot_name_prefix_ + str(self.turtlebot_counter_)

        self.spawn(x_,y_,name_)

    def spawn(self, x_, y_, turtlebot_name_):
        spawn_client = self.create_client(SpawnEntity, "/spawn_entity")
        while not spawn_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server...")        

        sdf_file_path = os.path.join(get_package_share_directory("turtlebot3_gazebo"), "models", "turtlebot3_burger", "model.sdf")

        request = SpawnEntity.Request()
        request.name = turtlebot_name_
        request.xml = open(sdf_file_path, 'r').read()
        request.robot_namespace = turtlebot_name_ + "_namespace"
        request.initial_pose.position.x = x_
        request.initial_pose.position.y = y_
        request.initial_pose.position.z = 0.01

        spawn_future = spawn_client.call_async(request)
        spawn_future.add_done_callback(partial(self.callback_spawn, x_=x_, y_=y_, turtlebot_name_=turtlebot_name_))

    def callback_spawn(self, spawn_future, x_, y_, turtlebot_name_):
        try:
            response = spawn_future.result()    # response.status_message
            if response.success == True:
                self.get_logger().info("Spawned " + turtlebot_name_ + " at:\nx: " + str(x_) + "\ny: " + str(y_))
                new_turtle = Turtle()
                new_turtle.turtle_name = turtlebot_name_
                new_turtle.x_pos = x_
                new_turtle.y_pos = y_
                self.alive_turtles_.append(new_turtle)
                self.publish_alive_turtles()
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
