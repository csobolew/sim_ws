import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import numpy as np

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.subscriber_ = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.last_ranges = np.zeros((1081,), dtype = np.float32)
        self.ranges = np.zeros((1081,), dtype = np.float32)

    def preprocess_lidar(self, ranges):
        temp_ranges = np.array(ranges)
        self.ranges = (temp_ranges+self.last_ranges)/2
        self.last_ranges = temp_ranges

        filter_index =  self.ranges > 1.4
        self.ranges[filter_index] = 1.4

    def find_max_gap(self, free_space_ranges):
        split_index = np.where(free_space_ranges == 0.0)[0]
        splits = np.split(free_space_ranges, split_index)
        split_lens = np.array([len(x) for x in splits])
        max_len_index = np.argmax(split_lens)
        if max_len_index == 0:
            start_gap = 0
            end_gap = split_lens[0] - 1
        else:
            start_gap = np.sum(split_lens[:max_len_index])
            end_gap = start_gap+split_lens[max_len_index]-1
        max_splits = splits[max_len_index]
        return start_gap, end_gap, max_splits

    def find_best_point(self, start_gap, end_gap, ranges):
        idx_list = np.where(ranges == np.max(ranges))[0]
        best_index = start_gap + idx_list[round(len(idx_list)/2)]
        return best_index

    def laser_callback(self, msg):
        ranges = msg.ranges
        self.preprocess_lidar(ranges)

        closest_point_index = np.argmin(self.ranges)
        bubble_size = 80
        lower = 0
        upper = len(ranges)-1
        if closest_point_index - bubble_size >= 0:
            lower = closest_point_index - bubble_size
        if closest_point_index + bubble_size <= len(ranges)-1:
            upper = closest_point_index + bubble_size
        bubble_index = np.array(range(lower, upper))
        self.ranges[bubble_index] = 0
        start_gap, end_gap, max_splits = self.find_max_gap(self.ranges)
        best_index = self.find_best_point(start_gap, end_gap, max_splits)
        angle_array = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        angle = angle_array[best_index]


        mymsg = AckermannDriveStamped()
        mymsg.drive.steering_angle = angle
        if abs(mymsg.drive.steering_angle) >= np.radians(0) and abs(mymsg.drive.steering_angle) < np.radians(10):
            mymsg.drive.speed = 2.0
        elif abs(mymsg.drive.steering_angle) >= np.radians(10) and abs(mymsg.drive.steering_angle) < np.radians(20):
            mymsg.drive.speed = 1.0
        else:
            mymsg.drive.speed = 0.5

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