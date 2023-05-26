import rclpy.node
import serial 
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import numpy as np
import math
from roboteq_constants import *

# For roboteq commands.
MAX_RUNTIME_COMMANDS_LENGTH = 3
MAX_RUNTIME_QUERIES_LENGTH = 3
MAX_MAINTENANCE_COMMAND_LENGTH = 3

LEFT_PORT = '/dev/left_roboteq'
RIGHT_PORT = '/dev/right_roboteq'
TIMEOUT = 5
STOP_COMMANDS = f"!G 1 0\r!G 2 0\r"


class Roboteq_Node(rclpy.node.Node):

    def __init__(self):
        super().__init__(node_name='roboteq_control_node')

        self.declare_parameter('cmdvel_topic', 'cmd_vel')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('tf_topic', '')


        self.declare_parameter('left_roboteq_port', LEFT_PORT)
        self.declare_parameter('right_roboteq_port', RIGHT_PORT)
        self.declare_parameter('baud', 115200)

        self.declare_parameter('wheel_circumference', 0.55)
        self.declare_parameter('wheel_radius',0.0875)
        self.declare_parameter('track_width', 0.445)

        self.motor_count = 2

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
            msg_type= Odometry,
            topic = self.get_parameter('odom_topic').get_parameter_value().string_value,
            qos_profile = 10
            )
        
        self.rel_time = self.get_clock().now() # use this to calculate delta_t for odom
        self.timer = self.create_timer(.001, self.generate_odom_and_tf)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.connect_serial()

    def generate_odom_and_tf(self):

        """
        Get the velocity info from roboteqs. Then calculate pose change. Add to pose and publish.
        Then broadcast the TF.
        """
       # Get all four wheel RPMS
        motors_rpm_command = RUNTIME_QUERIES.get('Read Encoder Motor Speed in RPM') # This turns into the letter 'S'.
        wheel_l_1, wheel_l_2 = self.write_runtime_query(motors_rpm_command) # What side? What front back. Don't know. Come back.
       
        def publish_odom(self, x, y, z, quat_x, quat_y, quat_z, quat_w):

            odom_message = Odometry()
            odom_message.header.stamp = self.get_clock().now().to_msg()

            odom_message.pose.pose.position.x = x
            odom_message.pose.pose.position.y = y
            odom_message.pose.pose.position.z = z 

            odom_message.pose.pose.orientation.x = quat_x
            odom_message.pose.pose.orientation.y = quat_y
            odom_message.pose.pose.orientation.z = quat_z
            odom_message.pose.pose.orientation.w = quat_w

            self.odom_pub.publish(odom_message)
            
            # Publish the TF here.
            
            


    # Write commands to roboteq.
    def write_runtime_command(self, cmd_str: str, cmd_vals: list[str]):
        runtime_char = '!'
        if len(cmd_str) > MAX_RUNTIME_COMMANDS_LENGTH:
            Exception("Invalid command length for runtime commands.")
        motor_cmd_string = ''
        for motor_num in range(self.motor_count):
            motor_cmd_string += f'{runtime_char}{cmd_str} {motor_num+1} {cmd_vals[motor_num]}\r'
        self.left_port.write(motor_cmd_string.encode())
        self.right_port.write(motor_cmd_string.encode())


    def write_runtime_query(self, cmd_str: str):
        query_char = '?'
        if len(cmd_str) > MAX_RUNTIME_QUERIES_LENGTH:
            Exception("Invalid command length for runtime queries.")
        #motor_cmd_string = ''
        #for motor_num in range(self.motor_count):
        #    motor_cmd_string += f'{query_char}{cmd_str} {motor_num+1}\r'

        motor_cmd_string1 = '?S 1\r'
        motor_cmd_string2 = '?S 2\r'
        self.write_runtime_command('G',[1000,1000])

        self.left_port.write(motor_cmd_string1.encode())
        left_mot_1 = self.left_port.read_until(b"\r")
        self.left_port.write(motor_cmd_string2.encode())
        left_mot_2 = self.left_port.read_until(b"\r")
        self.right_port.write(motor_cmd_string1.encode())
        right_mot_1 = self.left_port.read_until(b"\r")
        self.right_port.write(motor_cmd_string2.encode())
        right_mot_2 = self.left_port.read_until(b"\r")

     
        #print(right_mot_2)
        #y = self.right_port.write(motor_cmd_string.encode())
        
        return ([left_mot_1.decode(), left_mot_2.decode(), right_mot_1.decode(), right_mot_2.decode()])
        #return (self.left_port.write(motor_cmd_string.encode())), (self.right_port.write(motor_cmd_string.encode()))


    def write_maintenance_command(self, cmd_str: str):
        maint_char = '%'
        safety_key = '321654987' 
        if len(cmd_str) > MAX_MAINTENANCE_COMMAND_LENGTH:
            Exception("Invalid command length for maintenance commands.")
        motor_cmd_string = ''
        for motor_num in range(self.motor_count):
            motor_cmd_string += f'{maint_char}{cmd_str} {safety_key} \r'
        
        print(self.left_port.read_until(b"r"))
        print(self.right_port.read_until(b"r"))

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
    speed_list = []
    for each in range(600):
        speed_val = roboteq_node.write_runtime_query('S')
        speed_list.append(speed_val)
    for each in speed_list:
        print(each)
    #print(speed_list)
    #print(type(x))
    #print(type(y))
    #print(str(x))
    #print(str(y))
    #print(str(roboteq_node.write_runtime_query('S')))
    # roboteq_node.get_logger().info('waiting to recieve')

   #rclpy.spin(roboteq_node)

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