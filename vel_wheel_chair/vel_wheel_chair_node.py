import rclpy
import socket
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelWheelChairNode(Node):
    def __init__(self):
        super().__init__('vel_wheel_chair_node')
        self.max_metres_per_second = 1.0 # max velocity of wheelchair
        self.max_value_muc = 4095 # max value of motor unit controller
        self.wheel_radius = 0.254 # radius of wheel in metres
        self.width_between_wheels = 0.65 # in metres
        
        self.frequency = 1/100
        
        # timeout
        self.setTimeout = 70 # self.frequency * 70 = 0.7 seconds
        
        
        # udp socket
        self.UDP_IP = "10.1.111.112" #10.1.111.112
        self.UDP_PORT = 8888
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
        
        # conver to percentage
        percent_l = self.max_value_muc / 100 * self.drives['left']
        percent_r = self.max_value_muc / 100 * self.drives['right']
        
        drive_l = self.max_value_muc - (self.max_value_muc * (100 - percent_l) / 100)
        drive_r = self.max_value_muc - (self.max_value_muc * (100 - percent_r) / 100)
        
        if drive_l > self.max_value_muc:
            drive_l = self.max_value_muc
        if drive_r > self.max_value_muc:
            drive_r = self.max_value_muc
        
        self.get_logger().info('left: ' + str(drive_l) + ' right: ' + str(drive_r))
        message = str(int(drive_l)) + ',' + str(int(drive_r)) + '\n'
        self.sock.sendto(message.encode(), (self.UDP_IP, self.UDP_PORT))
    
def main(args=None):
    rclpy.init(args=args)
    vel_wheel_chair_node = VelWheelChairNode()
    rclpy.spin(vel_wheel_chair_node)
    vel_wheel_chair_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()