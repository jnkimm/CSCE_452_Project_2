import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from turtlesim.srv import Spawn,TeleportAbsolute
from std_srvs.srv import Empty
from rcl_interfaces.srv import SetParameters


def spawn_request(node,client,x,y,theta,name):
    # client = node.create_client(Spawn,"spawn_client")
    while not client.wait_for_service(1.0):
       node.get_logger().warn("Waiting for service...")
    
    request = Spawn.Request()
    request.x = x
    request.y = y
    request.theta = theta
    request.name = name

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node,future)
    
    response = future.result()
    if response is not None:
        node.get_logger().info('Turtle "{}" spawned succesfully'.format(response.name))
    else:
        node.get_logger().warn('Failed to spawn "{}"'.format(request.name))

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("Draw")
    #Have to create client in the main function for some reason
    clear_client = node.create_client(Empty,'clear')
    clear_request = Empty.Request()
    clear_future = clear_client.call_async(clear_request)
    rclpy.spin_until_future_complete(node,clear_future)

    # maroon_client = node.create_client(SetParameters,'background_color')
    # maroon_request = SetParameters.Request()
    # red_param = Parameter('background_r',Parameter.Type.INTEGER,80)
    # green_param = Parameter('background_g',Parameter.Type.INTEGER,0)
    # blue_param = Parameter('background_b',Parameter.Type.INTEGER,0)
    # maroon_request.parameters = {red_param,green_param,blue_param}
    # maroon_future = maroon_client.call_async(maroon_request)
    # rclpy.spin_until_future_complete(node,maroon_future)


    spawn_client = node.create_client(Spawn,'spawn')
    spawn_request(node,spawn_client,0.0,0.0,180.0,"Turtle_M")
    
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()