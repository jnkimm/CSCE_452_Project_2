import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from turtlesim.srv import TeleportAbsolute
from std_srvs.srv import Empty
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class Turtle(Node):
    def __init__(self,x,y,theta,points):
        super().__init__('Turtle')
        self.x = x
        self.y = y
        self.theta = theta
        self.points = points
        self.sum_error_angle = 0
        self.last_error_angle = 0 
        self.sum_error_linear = 0
        self.last_error_linear = 0 
        self.current_dest = 0
        self.checkpoint_reached = False
        self.change_angle = True
        self.move_x = False
        self.publisher = self.create_publisher(Twist,'/turtle1/cmd_vel',10)
        self.create_subscription(Pose,'/turtle1/pose',self.updateturtlepose,10)
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.move)
    
    def updateturtlepose(self,msg):
        self.x = msg.x
        self.y = msg.y
        if (msg.theta < 0):
            self.theta = (2*math.pi)+msg.theta
        else :
            self.theta = msg.theta
        rise = self.points[self.current_dest][1] - self.y
        run = self.points[self.current_dest][0] - self.x
        distance = abs(math.sqrt((rise**2)+(run**2)))
        if (run == 0):
            if (rise < 0):
                angle = 4.7123
            elif (rise > 0):
                angle = 1.5708
            else:
                angle = 0
        else:
            angle = math.atan(rise/run)
        if (angle < 0):
            angle += 2*math.pi
        if (abs(self.theta - angle) < 0.0005):
            self.change_angle = False
            self.move_x = True
        if (distance < 0.0005):
            self.move_x = False
            # self.change_angle = True;
        self.get_logger().info("Pose sensed: \n x:"+str(self.x)+"\n y:"+str(self.y)+"\n theta:"+str(self.theta)+"\n")
        
    def move(self):
        msg = Twist()
        rise = self.points[self.current_dest][1] - self.y
        run = self.points[self.current_dest][0] - self.x
        distance = abs(math.sqrt((rise**2)+(run**2)))
        if (run == 0):
            if (rise < 0):
                angle = 4.7123
            elif (rise > 0):
                angle = 1.5708
            else:
                angle = 0
        elif(run > 0):
            angle = math.atan(rise/run)
        else:
            
        # if (angle < 0):
        #     angle += 2*math.pi
        if (angle )
        if (self.change_angle):
            msg.angular.z = self.PID_Angle_controller(0.5,0.0,0.0,(self.theta-angle))
            print("message being sent"+str(msg.angular.z))
            # self.last_error = msg.angular.z
            self.publisher.publish(msg)
        if (self.move_x):
            msg.linear.x = self.PID_Linear_controller(0.5,0.0,0.0,distance)
            self.last_error_linear = msg.linear.x
            self.publisher.publish(msg)
            # while(i[0] != self.y && i[1] != self.x):

    def PID_Angle_controller(self,Kp,Ki,Kd,current):
        self.sum_error_angle += current
        u = (Kp*current) + (Ki*self.timer_period*self.sum_error_angle) + Kd*((current-self.last_error_angle)/self.timer_period)
        return u

    def PID_Linear_controller(self,Kp,Ki,Kd,current):
        u = (Kp*current) + (Ki*self.timer_period*self.sum_error_linear) + Kd*((current-self.last_error_linear)/self.timer_period)
        self.sum_error_angle += current
        return u

def main(args=None):
    rclpy.init(args=args)
    node = Turtle(5.4,5.4,0.0,[[5,5]])
    rclpy.spin(node)
    # tele_client = node.create_client(TeleportAbsolute,'/turtle1/teleport_absolute')
    # while not tele_client.wait_for_service(1.0):
    #    node.get_logger().warn("Waiting for service...")
    # tele_request = TeleportAbsolute.Request()
    # tele_request.x = 5.0
    # tele_request.y = 5.0
    # tele_request.theta = float((45*math.pi)/180)
    # tele_future = tele_client.call_async(tele_request)
    # rclpy.spin_until_future_complete(node,tele_future)

    # turtle_pub = node.create_publisher(Twist,'/turtle1/cmd_vel',10)
    # node.create_subscription(Pose,'/turtle1/pose',node.updateturtlepose,10)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()