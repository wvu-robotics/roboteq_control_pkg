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

import math
import serial 
import numpy as np
import rclpy.node
import time

from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

# from .roboteq_serial_port import RoboteqSerialPort
from .roboteq_constants import *

# For roboteq commands.
MAX_RUNTIME_COMMANDS_LENGTH = 3
MAX_RUNTIME_QUERIES_LENGTH = 3
MAX_MAINTENANCE_COMMAND_LENGTH = 3

LEFT_PORT = '/dev/left_roboteq'
RIGHT_PORT = '/dev/right_roboteq'
TIMEOUT = 5

MAX_MOTOR_COUNT = 3 
MAX_RUNTIME_COMMANDS_LENGTH = 3 
MAX_RUNTIME_QUERIES_LENGTH = 3
MAX_MAINTENANCE_COMMAND_LENGTH = 5


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
        self.declare_parameter('wheel_radius',0.125)
        self.declare_parameter('track_radius', 0.445)
        self.declare_parameter('track_width', 2*0.445)

        self.left_roboteq = RoboteqSerialPort(
            port= self.get_parameter('left_roboteq_port').get_parameter_value().string_value,
            baudrate= self.get_parameter('baud').get_parameter_value().integer_value,
            timeout= TIMEOUT,
            motor_count= 2
        )

        self.right_roboteq = RoboteqSerialPort(
            port= self.get_parameter('right_roboteq_port').get_parameter_value().string_value,
            baudrate= self.get_parameter('baud').get_parameter_value().integer_value,
            timeout= TIMEOUT,
            motor_count= 2
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
    
        self.timer = self.create_timer(.01, self.generate_odom_and_tf)
        self.rel_time = self.get_clock().now().nanoseconds 

        self.tf_broadcaster = TransformBroadcaster(self)

        self.left_roboteq.connect_serial()
        self.right_roboteq.connect_serial()

        # Initial Position
        self.x_pos = 0
        self.y_pos = 0
        self.theta = 0

    def generate_odom_and_tf(self):
        """
        Get the velocity info from roboteqs. Then calculate pose change. Add to pose and publish.
        Then broadcast the TF.
        """
        # Get all four wheel RPMS
        rpm_cmd = RT_RUNTIME_QUERIES.Read_Encoder_Motor_Speed_in_RPM
                
        # Get the average of the RPM for each side.
        # read_runtime_query returns list of string, eval(rpm) converts list elements to int from str
        rpm_query_output = self.left_roboteq.read_runtime_query(rpm_cmd) + self.right_roboteq.read_runtime_query(rpm_cmd)

        # Set the class-wide query string.

        print("rpm_query_output" + str(rpm_query_output))
        # Want to place the delta time calculation close to the runtime query to ensue that the time and measurement are as close as possible
        delta_time = (self.get_clock().now().nanoseconds - self.rel_time)/ 1e9

        try:
            rpm_values = list(map(int,rpm_query_output))
        except Exception as exception:
            rpm_values = [0,0,0,0]
            print("Ignoring rpm output: \n" + str(rpm_query_output) + "\n" + str(exception))

        left_rpms = rpm_values[0:1]
        right_rpms = rpm_values[2:3]

        print("----------\nLeft RPMS: " + str(left_rpms) + "\nRight RPMS: " + str(right_rpms))

        omega_l_avg = sum(left_rpms)/len(left_rpms)
        omega_r_avg = sum(right_rpms)/len(right_rpms)

        # Get the translational velocities for each side (m/s)
        V_r = omega_r_avg * 2 * math.pi * self.get_parameter('wheel_radius').get_parameter_value().double_value
        V_l = omega_l_avg * 2 * math.pi * self.get_parameter('wheel_radius').get_parameter_value().double_value

        # Get the total linear velocity of the robot body using both sides.
        V_avg = (V_r + V_l) / 2


        delta_theta = (V_r - V_l) / self.get_parameter('track_width').get_parameter_value().double_value * delta_time

        self.theta += delta_theta
        self.get_logger().info("THETA IS " + str(self.theta))

        # Get the components of translational velocity using our current theta.
        V_x = V_avg * math.cos(self.theta)
        V_y = V_avg * math.sin(self.theta)

        # Now find our pose change using the velocities
        X_delta = V_x * delta_time
        Y_delta = V_y * delta_time

        # Now update our position.
        self.x_pos += X_delta
        self.y_pos += Y_delta
        
        quats = self.quaternion_from_euler(0,0, math.radians(self.theta))

        # update the relative time and then publish odom message
        self.rel_time = int(self.get_clock().now().nanoseconds)
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


    def cmd_vel_callback(self, twist_msg: Twist):

        # Try calling this inside here to see if it solves the simultaneous access issue with the serial port.
        

        self.get_logger().info('Recieved twist message: \n' + str(twist_msg))

        track_width: float = self.get_parameter('track_width').get_parameter_value().double_value
        wheel_circumference: float= self.get_parameter('wheel_circumference').get_parameter_value().double_value

        right_speed = twist_msg.linear.x + 2 * twist_msg.angular.z * (track_width/2) # meters / second
        left_speed = twist_msg.linear.x - 2 * twist_msg.angular.z * (track_width/2) # meters / second

        right_rpm = (10 * right_speed / wheel_circumference) * 60 # m/s / rotations/m = rotations/sec * 60 = rotations/minute
        left_rpm = (10 * left_speed / wheel_circumference ) * 60 # m/s / rotations/m = rotations/sec * 60 = rotations/minute

        self.get_logger().info('Writing to serial ports')
        self.get_logger().info('left: ' + str(left_rpm))
        self.get_logger().info('right: ' + str(right_rpm))


        if(self.left_roboteq.is_open and self.right_roboteq.is_open):

            try:
                rpm_cmd = RT_RUNTIME_COMMANDS.Go_to_Speed_or_to_Relative_Position
                self.left_roboteq.write_runtime_command(rpm_cmd, [left_rpm, left_rpm])
                self.right_roboteq.write_runtime_command(rpm_cmd, [right_rpm, right_rpm])

            except Exception as serExcpt:
                self.get_logger().warn(str(serExcpt))
        time.sleep(.005)
        #self.generate_odom_and_tf()
"""






























































"""
class RoboteqSerialPort(serial.Serial):
    '''This class is used to abstract a serial port to a roboteq controller serial port 

        class is a super class of serial

        Attributes
        ----------
        motor_count : int
            The number of motors connected to the downstream roboteq controller
        self: serial
            The serial port that corresponds to the roboteq controller's serial port 

        Methods
        -------
        write_runtime_command(self, cmd_str: str, cmd_vals: list[str])

        read_runtime_query(self, cmd_str: str)

        write_maintenance_command(self, cmd_str: str)
    '''

    def __init__(self, port, baudrate, timeout, motor_count):
        super(RoboteqSerialPort,self).__init__(
            port= port,
            baudrate= baudrate,
            timeout= timeout,
        )
        if motor_count > MAX_MOTOR_COUNT:
            Exception("Invalid number of motors for Roboteq")
        self.motor_count = motor_count


    def write_runtime_command(self, cmd_str: str, cmd_vals: list[str]):
        '''This function writes runtime commands to each of the motors that correspond to a serial port/roboteq controller
    
            Parameters
            ----------
            cmd_str : str
                String of the serial command to be ran that corresponds to a roboteq runtime command
                
            cmd_vals : list[str]
                The values to be used in the serial command in the order of the motor numbers

            Returns
            -------
            None
        '''
        if(self.is_open):
            self.reset_output_buffer()
            runtime_char = '!'
            if len(cmd_str) > MAX_RUNTIME_COMMANDS_LENGTH:
                Exception("Invalid command length for runtime commands.")
            motor_cmd_string = ''
            for motor_num in range(self.motor_count):
                motor_cmd_string += f'{runtime_char}{cmd_str} {motor_num+1} {cmd_vals[motor_num]}\r'
            self.write(motor_cmd_string.encode())


    def read_runtime_query(self, cmd_str: str):
        '''This function reads the result of runtime queries and returns them in an list in order of motor number
        
            Parameters
            ----------
            cmd_str : str
                String of the serial command to be ran that corresponds to a roboteq runtime query

            Returns
            -------
            str[]: A list of strings giving the value of the query in the order of the motor numbers 
        '''
        if(self.is_open):
            self.reset_input_buffer()
            query_char = '?'
            if len(cmd_str) > MAX_RUNTIME_QUERIES_LENGTH:
                Exception("Invalid command length for runtime queries.")
            query_returns: list[str]= [] 
            # removing unvalid characters from the read byte string
            for motor_num in range(self.motor_count):
                # Creating the serial command in the correct format and writing to the port
                motor_query_string = f'{query_char}{cmd_str} {motor_num+1}\r'
                self.write(motor_query_string.encode())
                # Reading the serial port, replacing invalid characters 
                read_string = self.read_until(b'\r').decode()
                read_string = read_string.replace(f'{cmd_str}=','')
                read_string = read_string.replace('\r','')
                # Placing edited string in list in the order of the motors 
                query_returns.append(read_string)
            return query_returns
    

    def write_maintenance_command(self, cmd_str: str):
        '''This function writes maintenance commands to each of the motors that correspond to a serial port/roboteq controller
            
            Parameters
            ----------
            cmd_str : str
                String of the serial command to be ran that corresponds to a roboteq maintenance command

            Returns
            -------
            str[]: A list of strings giving the value of the query in the order of the motor numbers 
        '''
        if(self.is_open):

            maint_char = '%'
            safety_key = 321654987
            if len(cmd_str) > MAX_MAINTENANCE_COMMAND_LENGTH:
                Exception("Invalid command length for maintenance commands.")
    
            motor_maint_cmd_string = f'{maint_char}{cmd_str} {safety_key} \r'
            self.write(motor_maint_cmd_string.encode())


    def connect_serial(self):
        try:
            self.open()
            if (self.is_open):
                return True
        except serial.SerialException as serExcpt:
            print(str(serExcpt))


    def disconnect_serial(self):
        try:
            self.close()
            if (not self.is_open):
                return True
        except serial.SerialException as serExcpt:
            print(str(serExcpt))


def main(args=None):

  

    rclpy.init(args=args)

    roboteq_node = Roboteq_Node()
    # while True:
        # time.sleep(0.5)
    roboteq_node.generate_odom_and_tf()
    rclpy.spin(roboteq_node)

    roboteq_node.left_roboteq.disconnect_serial()
    roboteq_node.right_roboteq.disconnect_serial()

    roboteq_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


'''
Need:





'''