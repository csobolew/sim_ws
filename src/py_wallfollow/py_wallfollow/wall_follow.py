import rclpy
from rclpy.node import Node
import time
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import math

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 1000)
        self.subscriber_ = self.create_subscription(LaserScan, '/scan', self.laser_callback, 1000)
        self.prev_tmoment = time.time()
        self.integral = 0.0
        self.prev_error = 0.0

    def laser_callback(self, msg):
        b_indx = math.floor((90.0 / 180.0 * math.pi - msg.angle_min) / msg.angle_increment)
        b_angle = 90.0 / 180.0 * math.pi
        a_angle = 45.0 / 180.0 * math.pi
        a_indx = 0
        if (msg.angle_min > 45.0 / 180.0 * math.pi):
            a_angle = msg.angle_min
            a_indx = 0
        else:
            a_indx = math.floor((45.0 / 180.0 * math.pi - msg.angle_min) / msg.angle_increment)
        a_range = 0.0
        b_range = 0.0
        if not math.isnan(msg.ranges[a_indx]) and not math.isinf(msg.ranges[a_indx]):
            a_range = msg.ranges[a_indx]
        else:
            a_range = 100.0
        if not math.isnan(msg.ranges[b_indx]) and not math.isinf(msg.ranges[b_indx]):
            b_range = msg.ranges[b_indx]
        else:
            b_range = 100.0
        alpha = math.atan((a_range * math.cos(b_angle - a_angle) - b_range) / (a_range * math.sin(b_angle - a_angle)))
        d_t = b_range * math.cos(alpha)
        d_t1 = d_t + 1.00 * math.sin(alpha)
        error = 0.8 - d_t1
        mymsg = AckermannDriveStamped()
        t_moment = time.time()
        del_time = t_moment - self.prev_tmoment
        self.integral += self.prev_error * del_time
        mymsg.drive.steering_angle = -(1.00 * error + 0.001 * (error - self.prev_error) / del_time + 0.005 * self.integral)
        self.prev_tmoment = t_moment
        if (abs(mymsg.drive.steering_angle) > 20.0 / 180.0 * math.pi):
            mymsg.drive.speed = 0.25
        elif (abs(mymsg.drive.steering_angle) > 10.0 / 180.0 * math.pi):
            mymsg.drive.speed = 0.50
        else:
            mymsg.drive.speed = 1.0

        self.publisher_.publish(mymsg)
        self.get_logger().info('Publishing: "%s"' % mymsg.drive)


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()