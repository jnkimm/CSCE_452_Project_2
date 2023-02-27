#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class aggieLogoNode(Node):

    def __init__(self):
        super().__init__("aggie_logo_subscriber")
        self.get_logger().info("Aggie logo subscriber has been started")
        self.pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10
        )
        self.cmd_vel_publisher = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10
        )
        self.goals = [[None,None,math.pi], [2.0,None,None], [None,None,math.pi/2], [None,9.0,None], [None,None,0.0], [9.0,None,None], [None,None,-1*math.pi/2], [None,7.0,None], [None,None,-1*math.pi], [7.5,None,None], [None,None,math.pi/2], [None,7.8,None], [None,None,math.pi], [6.1,None,None], [None,None,-1*math.pi/2], [None,3.0,None], [None,None,0.0], [7.0,None,None], [None,None,-1*math.pi/2], [None,1.6,None], [None,None,-1*math.pi], [4.0,None,None], [None,None,math.pi/2], [None,3.0,None], [None,None,0.0], [4.9,None,None], [None,None,math.pi/2], [None,7.8,None], [None,None,math.pi], [3.4,None,None], [None,None,-1*math.pi/2], [None,7.0,None], [None,None,-1*math.pi], [2.0,None,None]]
        self.index = 0
        self.goal = self.goals[self.index]
        self.Kp = 1.0
        self.Ki = 1.0
        self.Kd = 1.0
        self.delta_t = 0.017
        self.current_error = 0.0
        self.integral_term = 0.0
        self.previous_error = 0.0
        self.goal = 10.0

    def pose_callback(self, pose: Pose):
        cmd = Twist()
        if self.goals[self.index][0]:
            self.goal = self.goals[self.index][0]
            self.current_error = self.goal - pose.x
            if abs(self.current_error) < 0.01:
                cmd.linear.x = 0.0
                self.cmd_vel_publisher.publish(cmd)
                #self.get_logger().info("x value: " + str(pose.x))
                if self.index + 1 == len(self.goals):
                    rclpy.shutdown()
                else:
                    self.index += 1
                    self.integral_term = 0.0
                    self.previous_error = 0.0
            else:
                proportional_term = self.Kp * self.current_error
                #self.get_logger().info("proportional_term: " + str(proportional_term))
                self.integral_term += self.Ki * self.current_error * self.delta_t
                #self.get_logger().info("integral_term: " + str(self.integral_term))
                derivative_term = self.Kd * ((self.current_error - self.previous_error) / self.delta_t)
                #self.get_logger().info("derivative term: " + str(derivative_term))
                self.previous_error = self.current_error
                control = proportional_term + self.integral_term + derivative_term
                #self.get_logger().info("control x: " + str(control) + "\n")
                cmd.linear.x = abs(control) / 10
                self.cmd_vel_publisher.publish(cmd)
        elif self.goals[self.index][1]:
            self.goal = self.goals[self.index][1]
            self.current_error = self.goal - pose.y
            if abs(self.current_error) < 0.01:
                cmd.linear.x = 0.0
                self.cmd_vel_publisher.publish(cmd)
                #self.get_logger().info("y value: " + str(pose.y))
                if self.index + 1 == len(self.goals):
                    rclpy.shutdown()
                else:
                    self.index += 1
                    self.integral_term = 0.0
                    self.previous_error = 0.0
            else:
                proportional_term = self.Kp * self.current_error
                #self.get_logger().info("proportional_term: " + str(proportional_term))
                self.integral_term += self.Ki * self.current_error * self.delta_t
                #self.get_logger().info("integral_term: " + str(self.integral_term))
                derivative_term = self.Kd * ((self.current_error - self.previous_error) / self.delta_t)
                #self.get_logger().info("derivative term: " + str(derivative_term))
                self.previous_error = self.current_error
                control = proportional_term + self.integral_term + derivative_term
                #elf.get_logger().info("control y: " + str(control) + "\n")
                cmd.linear.x = abs(control) / 10
                self.cmd_vel_publisher.publish(cmd)
        else:
            self.goal = self.goals[self.index][2]
            self.current_error = self.goal - pose.theta
            if abs(self.current_error) < 0.01:
                cmd.angular.z = 0.0
                self.cmd_vel_publisher.publish(cmd)
                #self.get_logger().info("theta value: " + str(pose.theta))
                if self.index + 1 == len(self.goals):
                    rclpy.shutdown()
                else:
                    self.index += 1
                    self.integral_term = 0.0
                    self.previous_error = 0.0
            else:
                proportional_term = self.Kp * self.current_error
                #self.get_logger().info("proportional_term: " + str(proportional_term))
                self.integral_term += self.Ki * self.current_error * self.delta_t
                #self.get_logger().info("integral_term: " + str(self.integral_term))
                derivative_term = self.Kd * ((self.current_error - self.previous_error) / self.delta_t)
                #self.get_logger().info("derivative term: " + str(derivative_term))
                self.previous_error = self.current_error
                control = proportional_term + self.integral_term + derivative_term
                #self.get_logger().info("control theta: " + str(control) + "\n")
                cmd.angular.z = control / 10
                self.cmd_vel_publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = aggieLogoNode()
    rclpy.spin(node)
    print("Welcome to the show")
    rclpy.shutdown()