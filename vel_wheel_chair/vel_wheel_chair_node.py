import rclpy
import socket
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelWheelChairNode(Node):
    def __init__(self):
        super().__init__('vel_wheel_chair_node')
        self.max_metres_per_second = 1.0 # max velocity of wheelchair
        self.max_value_muc = 4096 # max value of motor unit controller
        self.wheel_radius = 0.254 # radius of wheel in metres
        self.width_between_wheels = 0.65 # in metres
        
        self.frequency = 1/100
        
        # timeout
        self.setTimeout = 70 # self.frequency * 70 = 0.7 seconds
        
        
        # udp socket
        self.UDP_IP = "127.0.0.1"
        self.UDP_PORT = 5005
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        self.counting = 0
        
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        self.drives = {'left': 0, 'right': 0}
        
        # subscribe to cmd_vel
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.timer = self.create_timer(self.frequency, self.timer_callback)
        
    def cmd_vel_callback(self, Twist):
        self.linear_velocity = Twist.linear.x
        self.angular_velocity = Twist.angular.z
        
        # limit the velocity
        if self.linear_velocity > self.max_metres_per_second:
            self.linear_velocity = self.max_metres_per_second
        if self.angular_velocity > self.max_metres_per_second:
            self.angular_velocity = self.max_metres_per_second
            
        # calculate the velocity of each wheel
        self.drives['left'] = (self.linear_velocity - (self.angular_velocity * self.width_between_wheels / 2)) / self.wheel_radius
        self.drives['right'] = (self.linear_velocity + (self.angular_velocity * self.width_between_wheels / 2)) / self.wheel_radius
        self.counting = 0
        
    def timer_callback(self):
        # publish the velocity of each wheel
        if self.counting >= self.setTimeout:
            self.drives['left'] = 0
            self.drives['right'] = 0
        self.counting += 1
        
        self.get_logger().info('left: ' + str(self.drives['left']) + ' right: ' + str(self.drives['right']))
        message = str(int(self.drives['left'])) + ',' + str(int(self.drives['right'])) + '\n'
        self.sock.sendto(message.encode(), (self.UDP_IP, self.UDP_PORT))
        
    
def main(args=None):
    rclpy.init(args=args)
    vel_wheel_chair_node = VelWheelChairNode()
    rclpy.spin(vel_wheel_chair_node)
    vel_wheel_chair_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()