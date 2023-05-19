import rclpy.node
import serial 
from geometry_msgs.msg import Twist

LEFT_PORT = '/dev/left_roboteq'
RIGHT_PORT = '/dev/right_roboteq'
TIMEOUT = 5

STOP_COMMANDS = f"!G 1 0\r!G 2 0\r"


class Roboteq_Node(rclpy.node.Node):

    def __init__(self):
        super().__init__(node_name='roboteq_control_node')

        self.declare_parameter('cmdvel_topic', 'cmd_vel')
        self.declare_parameter('odom_topic', 'odom')

        self.declare_parameter('left_roboteq_port', LEFT_PORT)
        self.declare_parameter('right_roboteq_port', RIGHT_PORT)
        self.declare_parameter('baud', 115200)

        self.declare_parameter('wheel_circumference', 0.55)
        self.declare_parameter('wheel_radius',0.0875)
        self.declare_parameter('track_width', 0.445)

        self.left_port = serial.Serial(
            port = self.get_parameter('left_roboteq_port').get_parameter_value().string_value,
            baudrate = self.get_parameter('baud').get_parameter_value().integer_value,
            timeout= TIMEOUT,
        )
        self.right_port = serial.Serial(            
            port = self.get_parameter('right_roboteq_port').get_parameter_value().string_value,
            baudrate = self.get_parameter('baud').get_parameter_value().integer_value,
            timeout= TIMEOUT,
            )
        
        self.cmd_vel_sub = self.create_subscription(
            msg_type= Twist,
            topic = self.get_parameter('cmdvel_topic').get_parameter_value().string_value,
            callback = self.cmd_vel_callback,
            qos_profile = 1
            )
        
        self.odom_pub = self.create_publisher(
            msg_type= Twist,
            topic = self.get_parameter('odom_topic').get_parameter_value().string_value,
            qos_profile = 10
            )
        self.connect_serial()


    def connect_serial(self):

        self.get_logger().info('\nOpening these serial ports:\n   left roboteq port: %s \n   right roboteq port: %s \n   (baudrate = %s)\n' % 
            (
            self.get_parameter('left_roboteq_port').get_parameter_value().string_value,
            self.get_parameter('right_roboteq_port').get_parameter_value().string_value,
            self.get_parameter('baud').get_parameter_value().integer_value,
            ) 
            )
        try:
            self.left_port.open()
            self.right_port.open()

            if (self.left_port.is_open):
                self.get_logger().info('Successfully opened the left port')

            if (self.right_port.is_open):
                self.get_logger().info('Successfully opened the right port')
        
        except serial.SerialException as serExcpt:
            self.get_logger().warn('serial.SerialException: %s' % str(serExcpt))


    def disconnect_serial(self):

        self.get_logger().info('\nOpening these serial ports:\n   left roboteq port: %s \n   right roboteq port: %s \n   (baudrate = %s)\n' % 
            (self.get_parameter('left_roboteq_port').get_parameter_value().string_value,
            self.get_parameter('right_roboteq_port').get_parameter_value().string_value,
            self.get_parameter('baud').get_parameter_value().integer_value,
            ))
        
        try:
            self.left_port.close()
            self.right_port.close()

            if (not self.left_port.is_open):
                self.get_logger().info('Successfully closed the left port')

            if (not self.right_port.is_open):
                self.get_logger().info('Successfully closed the right port')
        
        except serial.SerialException as serExcpt:
            self.get_logger().warn('serial.SerialException: %s' % str(serExcpt))

    def configure_motor(self):

        self.get_logger().info('Configuring motor')


    # def read_values(self):


    def cmd_vel_callback(self, twist_msg: Twist):

        self.get_logger().info('Recieved twist message: \n' + str(twist_msg))

        track_width: float = self.get_parameter('track_width').get_parameter_value().double_value
        wheel_circumference: float= self.get_parameter('wheel_circumference').get_parameter_value().double_value

        right_speed = twist_msg.linear.x + 2 * twist_msg.angular.z * (track_width/2) # meters / second
        left_speed = twist_msg.linear.x - 2 * twist_msg.angular.z * (track_width/2) # meters / second

        right_rpm = (10 * right_speed / wheel_circumference) * 60 # m/s / rotations/m = rotations/sec * 60 = rotations/minute
        left_rpm = (10 * left_speed / wheel_circumference ) * 60 # m/s / rotations/m = rotations/sec * 60 = rotations/minute

        left_speed_cmd = f"!G 1 {left_rpm}\r!G 2 {left_rpm}\r"
        right_speed_cmd = f"!G 1 {right_rpm}\r!G 2 {right_rpm}\r"

        self.get_logger().info('Writing to serial ports')
        self.get_logger().info('left: ' + str(left_rpm))
        self.get_logger().info('right: ' + str(right_rpm))

        try:
            self.left_port.write(left_speed_cmd.encode())
            self.right_port.write(right_speed_cmd.encode())

        except serial.SerialException as serExcpt:
            self.get_logger().warn('serial.SerialException: %s' % str(serExcpt))

def main(args=None):

    rclpy.init(args=args)

    roboteq_node = Roboteq_Node()
    # roboteq_node.get_logger().info('waiting to recieve')

    rclpy.spin(roboteq_node)

    roboteq_node.disconnect_serial()

    roboteq_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


'''
cmd_vel_setup method
odom setup method
make odom publisher 
make cmd_vel subscriber 
cmd_vel_callback
'''