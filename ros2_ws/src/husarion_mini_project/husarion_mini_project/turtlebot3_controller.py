#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle

class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtlebot_controller")

        self.declare_parameter("closest", True)
        self.closest_ = self.get_parameter("closest").value

        self.target_ = None
        self.pose_ = None
        self.alive_turtles_ = []

        self.pose_sub_ = self.create_subscription(Odometry, '/odom', self.callback_pose, 10)
        self.vel_pub_ = self.create_publisher(Twist, '/cmd_vel',10)
        self.alive_sub_ = self.create_subscription(TurtleArray,"/alive_turtles",self.callback_alive_turtles, 10)
        self.timer_ = self.create_timer(0.05, self.navigate)

        self.get_logger().info("Turtlebot Controller has been started!")

    def callback_alive_turtles(self, msg):
        if len(msg.turtles_array) > 0:
            if self.closest_:
                self.alive_turtles_ = msg
                prev_distance = 20.0
                for (i, tur) in enumerate(self.alive_turtles_.turtles_array):
                    distance = math.sqrt((self.pose_.position.x - tur.x_pos)**2 + (self.pose_.position.y - tur.y_pos)**2)
                    if distance < prev_distance:
                        prev_distance = distance
                        closest_index = i
                self.target_ = self.alive_turtles_.turtles_array[closest_index]
            else:
                self.target_ = msg.turtles_array[0]

    def callback_pose(self, msg):
        self.pose_ = msg.pose.pose # .position .x .y .z | .orientation .x .y .z .w

    def navigate(self):
        if self.pose_ == None or self.target_ == None:
            return

        dy_ = self.target_.y_pos - self.pose_.position.y
        dx_ = self.target_.x_pos - self.pose_.position.x       
        d_ = math.sqrt(dy_**2 + dx_**2)

        order_ = Twist()

        if d_ > 0.4:
            order_.linear.x = 0.5*d_
            if order_.linear.x > 1.5:
                order_.linear.x = 1.5
            if order_.linear.x < 0.6:
                order_.linear.x = 0.6

            alpha_ = math.atan2(dy_, dx_)
            theta_ = self.theta_from_quaterion(self.pose_.orientation.x, self.pose_.orientation.y, self.pose_.orientation.z, self.pose_.orientation.w)
            diff = alpha_ - theta_
            if diff > math.pi:
                diff -= 2*math.pi
            elif diff < -math.pi:
                diff += 2*math.pi
            order_.angular.z = 1.0*diff
        else:
            self.call_catch_turtle_server(self.target_.turtle_name)
            self.target_ = None
            order_.linear.x = 0.0
            order_.angular.z = 0.0

        self.vel_pub_.publish(order_) 

    def theta_from_quaterion(self, x, y, z, w):
        a1 = 2.0 * (w * z + x * y)
        a2 = 1.0 - 2.0 * (y * y + z * z)
        theta = math.atan2(a1, a2)
        return theta # in radians

    def call_catch_turtle_server(self, turtle_name):
        client = self.create_client(CatchTurtle, "/catch_turtle")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server...")        

        request = CatchTurtle.Request()
        request.turtle_name = turtle_name
        
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_catch, turtle_name = turtle_name))

    def callback_catch(self, future, turtle_name):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error("Turtle " + str(turtle_name) + " could not be cought")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()