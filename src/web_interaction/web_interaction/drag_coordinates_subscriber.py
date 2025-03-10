import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class DragCoordinatesSubscriber(Node):
    def __init__(self):
        super().__init__('drag_coordinates_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/drag_coordinates',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        start_x, start_y, end_x, end_y = msg.data
        self.get_logger().info(f"Dragged from ({start_x}, {start_y}) to ({end_x}, {end_y})")

def main(args=None):
    rclpy.init(args=args)

    drag_coordinates_subscriber = DragCoordinatesSubscriber()

    rclpy.spin(drag_coordinates_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    drag_coordinates_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
