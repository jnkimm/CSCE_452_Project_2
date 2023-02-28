import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen

import math

# hard-coded points and thetas for the robot to move to
# THIS IS A SAMPLE ARRAY DELETE LATER!!!!!!!!!!!!!!!!!!!!!
A_points = [(1,3.4),(1,4),(1.4,4),(2.2,6),(2,6),(2,6.6),(3.4,6.6),              #
            (3.4,6),(3.2,6),(4,4),(4.4,4),(4.4,3.4),(3.2,3.4),(3.2,4),(3.4,4),  # Points of A
            (3.3,4.4),(2.1,4.4),(2,4),(2.2,4),(2.2,3.4),(1,3.4),                #
            (2.5,5),(2.7,5.5),(2.9,5),(2.5,5), # middle of AS
            (1,1)] # end point

class Draw(Node):
    def __init__(self, point_arr):
        super().__init__("draw")
        # Pub Sub
        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.timer_ = self.create_timer(0.01, self.move)
        self.get_logger().info("Started drawing A")
        # variables
        self.pose = Pose()
        self.point_counter = 0      # keep track of points in array
        self.points = point_arr
        # Change pen to off
        client_pen = self.create_client(SetPen,"/turtle1/set_pen")
        request = SetPen.Request()
        request.off = 1
        client_pen.call_async(request)

    def pose_callback(self, data=Pose()):
        self.pose = data
		
    def move(self):
        goal_x, goal_y = self.points[self.point_counter]
        lin_tol = 0.05
        angl_tol = 0.01
        msg = Twist()

        # math portion to find angle and distance
        k_lin = 0.5
        k_ang = 6.0
        dist = math.sqrt((goal_x-self.pose.x)**2 + (goal_y-self.pose.y)**2)
        angle = math.atan2((goal_y-self.pose.y),(goal_x-self.pose.x))
        
        # Move to angle first
        if abs(angle - self.pose.theta) > angl_tol:
            msg.angular.z = k_ang*(angle - self.pose.theta)
        # Then move to point
        else:
            if  dist >= lin_tol:
                msg.linear.x = k_lin*dist
            else:
                msg.linear.x = 0.0
                if self.point_counter == len(self.points)-2:                       # turn off pen
                    client_pen = self.create_client(SetPen,"/turtle1/set_pen")
                    request = SetPen.Request()
                    request.off = 1
                    client_pen.call_async(request)
                if self.point_counter == 0:                                     # turn on pen
                    client_pen = self.create_client(SetPen,"/turtle1/set_pen")
                    request = SetPen.Request()
                    request.r = 255
                    request.g = 255
                    request.b = 255
                    request.width = 3
                    request.off = 0
                    client_pen.call_async(request)
                if(self.point_counter + 1 == len(self.points)):
                    quit()
                self.point_counter += 1
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Draw(A_points)
    rclpy.spin(node)
    rclpy.shutdown()