import rclpy
from rclpy.node import Node
import threading
import sys
from select import select
import termios
import tty
from ackermann_msgs.msg import AckermannDriveStamped

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        timer_period = 0.005
        self.max_speed = 5
        self.speed_factor = 4.5
        self.steer_speed_factor = 4.5
        self.max_steer = 0.4189
        self.msg = AckermannDriveStamped()
        self.timer = self.create_timer(timer_period, self.timer_callback)
    def timer_callback(self):
        try:
            self.publisher_.publish(self.msg)
        except Exception as e:
            if str(e) == "publish() to a closed topic":
                pass
            else:
                raise e
        #self.get_logger().info('Publishing: "%s"' % self.msg.drive)
    def forward(self):
        self.send_drive_comm(self.max_speed/self.speed_factor, 0.0)
    def stop(self):
        self.send_drive_comm(0.0, 0.0)
    def right(self):
        self.send_drive_comm(self.max_speed/self.steer_speed_factor, -self.max_steer)
    def left(self):
        self.send_drive_comm(self.max_speed/self.steer_speed_factor, self.max_steer)
    def slight_right(self):
        self.send_drive_comm(self.max_speed/self.steer_speed_factor, -self.max_steer/2.0)
    def slight_left(self):
        self.send_drive_comm(self.max_speed/self.steer_speed_factor, self.max_steer/2.0)
    def send_drive_comm(self, speed, angle):
        self.last_angle = angle
        self.last_speed = speed
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = angle
        self.msg = msg


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select([sys.stdin], [], [], 0.005)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    return key
def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(controller)
    controller_thread = threading.Thread(target=executor.spin, daemon=False)
    controller_thread.start()
    settings = termios.tcgetattr(sys.stdin)
    while(True):
        key = getKey()
        if key == 'q':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            executor.shutdown()
            exit()
        if key == 'i':
            controller.forward()
        if key == 'k':
            controller.stop()
        if key == 'u':
            controller.slight_left()
        if key == 'o':
            controller.slight_right()
        if key == 'j':
            controller.left()
        if key == 'l':
            controller.right()

if __name__ == '__main__':
    main()