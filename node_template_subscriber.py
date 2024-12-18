#!/usr/bin/env python3
   
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class {{NODE_NAME}}(Node):
    def __init__(self):
        super().__init__('{{PKG_NAME}}_{{NODE_NAME}}')
        self.subscription = self.create_subscription(
            String,
            '{{PKG_NAME}}/{{NODE_NAME}}', 
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    try:
        rclpy.init(args=args)
        node = {{NODE_NAME}}()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass        
    except Exception as e:
        print(f"Exception: {e}")        

if __name__ == '__main__':
    main()      