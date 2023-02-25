import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn


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
    spawn_client = node.create_client(Spawn,'spawn')
    spawn_request(node,spawn_client,30.0,30.0,180.0,"Turtle_T")

    
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()