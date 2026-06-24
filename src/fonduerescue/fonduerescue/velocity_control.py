import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

DURATION = 5.0   # seconds
LIN_X = 1.0      # m/s forward
ANG_Z = 0.0      # rad/s

class GoalVelPub(Node):
    def __init__(self):
        super().__init__('goal_vel_pub')
        
        self.pub = self.create_publisher(TwistStamped, 'nav_vel', 10)
        self.t0 = self.get_clock().now()
        self.create_timer(0.02, self.tick)  # 50 Hz

    def _make_msg(self, linear_x=0.0, angular_z=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        return msg

    def tick(self):
        dt = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        if dt < DURATION:
            self.pub.publish(self._make_msg(LIN_X, ANG_Z))
        else:
            self.pub.publish(self._make_msg())
            self.get_logger().info(f'{DURATION}s elapsed → stop, shutdown')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = GoalVelPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
