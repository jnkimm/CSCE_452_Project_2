import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from std_srvs.srv import Empty
from turtlesim.srv import Spawn

import math

# hard-coded points and thetas for the robot to move to
# THIS IS A SAMPLE ARRAY DELETE LATER!!!!!!!!!!!!!!!!!!!!!
A_points = [(1.4366,3.5794),
            (1.4366,4.123),
            (1.752,4.123),
            (2.5247,5.884),
            (2.3523,5.884),
            (2.3523,6.444),
            (3.585,6.444),
            (3.585,5.884),
            (3.4126,5.884),
            (4.1853,4.123),
            (4.5007,4.123),
            (4.5007,3.5794),
            (3.384,3.5794),
            (3.384,4.123),
            (3.558,4.123),
            (3.427,4.423),
            (2.5103,4.423),
            (2.3793,4.123),
            (2.5533,4.123),
            (2.5533,3.5794),
            (1.4366,3.5794),
            (2,4)]




T_points = [(4.19,1.956),
            (4.19,1.956),
            (4.19,3.192),
            (4.88,3.192),
            (4.88,7.52),
            (3.579,7.52),
            (3.579,6.84),
            (2.341,6.84),
            (2.341,8.626),
            (8.671,8.626),
            (8.671,6.84),
            (7.433,6.84),
            (7.433,7.52),
            (6.132,7.52),
            (6.132,3.192),
            (6.822,3.192),
            (6.822,1.956),
            (4.19,1.956),
            (6.55,3.5794)]

M_points = [(6.55,3.5794),
            (6.55,3.5794),
            (6.55,4.123),
            (6.753,4.123),
            (6.753,5.884),
            (6.55,5.884),
            (6.55,6.444),
            (7.444,6.444),
            (8.057,5.213),
            (8.67,6.444),
            (9.564,6.444),
            (9.564,5.884),
            (9.361,5.884),
            (9.361,4.123),
            (9.564,4.123),
            (9.564,3.5794),
            (8.571,3.5794),
            (8.571,4.123),
            (8.774,4.123),
            (8.774,5.328),
            (8.057,3.886),
            (7.34,5.328),
            (7.34,4.123),
            (7.543,4.123),
            (7.543,3.5794),
            (6.55,3.5794),
            (1,1)]

Tri_points = [(2.765,4.996),
            (2.765,4.996),
            (3.167,4.996),
            (2.966,5.465),
            (2.765,4.996),
            (4.19,1.956)]

ar = [A_points,Tri_points,T_points,M_points]

class Draw(Node):
    def __init__(self,point_arr):
        super().__init__("draw")
        # Pub Sub
        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.timer_ = self.create_timer(0.01, self.move)
   
        # variables
        self.pose = Pose()
        self.point_counter = 0      # keep track of points in array
        self.points = point_arr
        self.letter_counter = 0
        # Change pen to off
        self.client_pen = self.create_client(SetPen,"/turtle1/set_pen")
        request = SetPen.Request()
        request.off = 1
        self.client_pen.call_async(request)

    def pose_callback(self, data=Pose()):
        self.pose = data
		
    def move(self):
        goal_x, goal_y = self.points[self.letter_counter][self.point_counter]
        lin_tol = 0.05
        angl_tol = 0.01
        msg = Twist()

        # math portion to find angle and distance
        k_lin = 6.0
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
                if self.point_counter == 1:
                    request = SetPen.Request()
                    request.r = 255
                    request.g = 255
                    request.b = 255
                    request.width = 3
                    request.off = 0
                    self.client_pen.call_async(request)
                if self.point_counter == len(self.points[self.letter_counter])-2:                       # turn off pen
                    # client_pen = self.create_client(SetPen,"/turtle1/set_pen")
                    request = SetPen.Request()
                    request.off = 1
                    self.client_pen.call_async(request)
                if self.point_counter == 0:                                     # turn on pen
                    # client_pen = self.create_client(SetPen,"/turtle1/set_pen")
                    request = SetPen.Request()
                    request.r = 255
                    request.g = 255
                    request.b = 255
                    request.width = 3
                    request.off = 0
                    self.client_pen.call_async(request)
                if(self.point_counter + 1 == len(self.points[self.letter_counter])):
                    self.letter_counter += 1
                    self.point_counter = 0
                    request = SetPen.Request()
                    request.off = 1
                    self.client_pen.call_async(request)
                self.point_counter += 1
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Draw(ar)
    clear_client = node.create_client(Empty,'clear')
    clear_request = Empty.Request()
    clear_future = clear_client.call_async(clear_request)

    rclpy.spin(node)
    rclpy.shutdown()  

if __name__ == '__main__':
    main()
