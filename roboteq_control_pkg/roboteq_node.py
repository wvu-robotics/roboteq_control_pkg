# generate_roboteq_constants.py
# By: Nathan Adkins 
# WVU IRL 

'''
Roboteq User Manual: https://www.roboteq.com/docman-list/motor-controllers-documents-and-files/documentation/user-manual/272-roboteq-controllers-user-manual-v21/file

Runtime Commands                              page 188  
DS402 Runtime Commands                        page 209  
Runtime Queries                               page 222  
DS402 Runtime Queries                         page 268  
Query History Commands                        page 286  
Maintenance Commands                          page 289  
General Configuration and Safety              page 296  
Analog, Digital, Pulse IO Configurations      page 312  
Motor Configurations                          page 330  
Brushless Secific Commands                    page 364  
AC Induction Specific Commands                page 385  
CAN Communication Commands                    page 391  
TCP Communication Commands                    page 397 
'''

import rclpy.node
import serial 
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import numpy as np

import math
from .roboteq_constants import *

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
        self.declare_parameter('track_radius', 0.445)
        self.declare_parameter('track_width', 2*0.445)

        self.motor_count = 2

        # Our position
        self.x_pos = 0
        self.y_pos = 0
        self.theta = 0


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
        self.timer = self.create_timer(.01, self.generate_odom_and_tf)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.connect_serial()

    def generate_odom_and_tf(self):

        """
        Get the velocity info from roboteqs. Then calculate pose change. Add to pose and publish.
        Then broadcast the TF.
        """
       # Get all four wheel RPMS

        delta_time = 0.01 # Come back to this

        motors_rpm_command = RUNTIME_QUERIES.get('Read Encoder Motor Speed in RPM') # This turns into the letter 'S'.
        wheel_velocities = self.write_runtime_query(motors_rpm_command) # What side? What front back. Don't know. Come back.



        # Get the average of the RPM for each side.
        omega_r_avg = sum(wheel_velocities[0:1]) / 2
        omega_l_avg = sum(wheel_velocities[2:3]) / 2

        # Get the translational velocities for each side (m/s)
        V_r = omega_r_avg * 2 * math.pi * self.get_parameter('wheel_radius').get_parameter_value().double_value
        V_l = omega_l_avg * 2 * math.pi * self.get_parameter('wheel_radius').get_parameter_value().double_value

        # Get the total linear velocity of the robot body using both sides.
        V_avg = (V_r + V_l) / 2

        delta_theta = (V_r - V_l) / self.get_parameter('track_width').get_parameter_value().double_value * delta_time

        self.theta += delta_theta

        # Get the components of translational velocity using our current theta.
        V_x = V_avg * math.cos(self.theta)
        V_y = V_avg * math.sin(self.theta)

        # Now find our pose change using the velocities
        X_delta = V_x * delta_time
        Y_delta = V_y * delta_time

        # Now update our position.
        self.x_pos += X_delta
        self.y_pos += Y_delta
        
        quats = self.quaternion_from_euler(0,0, self.theta)
        
        self.publish_odom(self.x_pos, self.y_pos, 0.0, quats) 
       
        # Publish the TF here.
            
            

    def publish_odom(self, x, y, z, quats):

        odom_message = Odometry()
        odom_message.header.stamp = self.get_clock().now().to_msg()

        odom_message.pose.pose.position.x = x
        odom_message.pose.pose.position.y = y
        odom_message.pose.pose.position.z = z 
        quat_message = Quaternion()
        quat_message.x = quats[0]
        quat_message.y = quats[1]
        quat_message.z = quats[2]
        quat_message.w = quats[3]

        odom_message.pose.pose.orientation = quat_message
                
        print("I am pubbing to odom dawg")
        self.odom_pub.publish(odom_message)


    def quaternion_from_euler(self, ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q


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
        left_mot_1 = self.left_port.read_until(b'\r').replace(b'+',b'0')
        self.left_port.write(motor_cmd_string2.encode())
        left_mot_2 = self.left_port.read_until(b'\r').replace(b'+',b'0')
        self.right_port.write(motor_cmd_string1.encode())
        right_mot_1 = self.left_port.read_until(b'\r').replace(b'S=+',b'0')
        self.right_port.write(motor_cmd_string2.encode())
        right_mot_2 = self.left_port.read_until(b'\r').replace(b'S=+',b'0')

     
        #print(right_mot_2)
        #y = self.right_port.write(motor_cmd_string.encode())
        
        return ([int(left_mot_1.decode()), int(left_mot_2.decode()), int(left_mot_1.decode()), int(left_mot_2.decode())])
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
    #speed_list = []
   # for each in range(6000):
   #     speed_val = roboteq_node.write_runtime_query('S')
   #     speed_list.append(speed_val)
   # for each in speed_list:
   #     print(each)
    #print(speed_list)
    #print(type(x))
    #print(type(y))
    #print(str(x))
    #print(str(y))
    #print(str(roboteq_node.write_runtime_query('S')))
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