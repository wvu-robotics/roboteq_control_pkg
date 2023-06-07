# roboteq_node.py
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
import numpy as np
import rclpy.node
import time

from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from .roboteq_serial_port import RoboteqSerialPort
from .roboteq_constants import *

# ROS CONSTANTS 
# -------------
DEFAULT_SUB_CMD_VEL_TOPIC = 'cmd_vel' # ROS topic this node subscribes to in the command velocity callback 
DEFAULT_PUB_ODOM_TOPIC = 'odom' # ROS topic this node publishes the odometry message to (nav_msgs.msg Odometry)

DEFAULT_ODOM_PUBLISH_RATE_HZ = 100.0 # Number of odom publishes per second
DEFAULT_COMMAND_VEL_CALLBACK_DELAY = 0.005 # Seconds forced between each call of the callback function 
# -------------


# DEVICE, MOTOR, AND INTRINSIC CONSTANTS
# -------------------------------------- 
MOTORS_PER_ROBOTEQ = 2 # Motors per Roboteq 

DEFAULT_LEFT_PORT = '/dev/left_roboteq' # Device path to the left roboteq motor controller
DEFAULT_RIGHT_PORT = '/dev/right_roboteq' # Device path to the right roboteq motor controller

DEFAULT_WHEEL_RADIUS = 0.125 # Radius of the wheels (METERS)
DEFAULT_TRACK_WIDTH = 0.77 # Distance between wheels on either side of chassis (Bilaterally) (METERS)

FUDGE_VALUE_NO_IDEA_WHY_THIS_WORKS_FIGURE_OUT_LATER_NEED_TO_MOVE_ON = 10
DEFAULT_GEAR_REDUCTION_RATIO = ((12.0/40.0) ** 3) * FUDGE_VALUE_NO_IDEA_WHY_THIS_WORKS_FIGURE_OUT_LATER_NEED_TO_MOVE_ON # Found by looking at the output gears on the gearbox

DEFAULT_BAUD = 115200 # Bits per second / pulses per second (Value acceptable by roboteqs)
DEFAULT_TIMEOUT = 5
# -------------------------------------- 


# SAFETY & MISC CONSTANTS 
# -----------------------
DEFAULT_MAX_METERS_PER_SECOND = 2.0 # Maximum meters per second 
DEFAULT_DEBUGGING = True 
# ----------------------




class Roboteq_Node(rclpy.node.Node):


    def __init__(self):
        super().__init__(node_name='roboteq_control_node')

        # ROS PARAMETERS 
        # ----------------------
        self.declare_parameter('odom_pub_rate_hz', DEFAULT_ODOM_PUBLISH_RATE_HZ)
        self.declare_parameter('cmd_vel_topic', DEFAULT_SUB_CMD_VEL_TOPIC)
        self.declare_parameter('odom_topic', DEFAULT_PUB_ODOM_TOPIC)
        self.declare_parameter('cmd_vel_delay_sec',DEFAULT_COMMAND_VEL_CALLBACK_DELAY)
        # ----------------------


        # ----------------------
        # DEVICE, MOTOR, AND INTRINSIC PARAMETERS
        self.declare_parameter('left_roboteq_port', DEFAULT_LEFT_PORT)
        self.declare_parameter('right_roboteq_port', DEFAULT_RIGHT_PORT)
        self.declare_parameter('gear_reduction_ratio', DEFAULT_GEAR_REDUCTION_RATIO)
        self.declare_parameter('wheel_radius', DEFAULT_WHEEL_RADIUS)
        self.declare_parameter('track_width', DEFAULT_TRACK_WIDTH) 
        # ----------------------


        # SAFETY & MISC PARAMETERS
        # ----------------------
        self.declare_parameter('debugging_state', DEFAULT_DEBUGGING)
        self.declare_parameter('max_linear_speed', DEFAULT_MAX_METERS_PER_SECOND)
        self.declare_parameter('max_rpm', (DEFAULT_MAX_METERS_PER_SECOND) * (60.0/1.0) / (2.0 * math.pi * DEFAULT_WHEEL_RADIUS) )
        # ----------------------


        self.left_roboteq = RoboteqSerialPort(
            port= self.get_parameter('left_roboteq_port').get_parameter_value().string_value,
            baudrate= DEFAULT_BAUD,
            timeout= DEFAULT_TIMEOUT,
            motor_count= MOTORS_PER_ROBOTEQ
        )

        self.right_roboteq = RoboteqSerialPort(
            port= self.get_parameter('right_roboteq_port').get_parameter_value().string_value,
            baudrate= DEFAULT_BAUD,
            timeout= DEFAULT_TIMEOUT,
            motor_count= MOTORS_PER_ROBOTEQ
        )
        
        self.cmd_vel_sub = self.create_subscription(
            msg_type= Twist,
            topic= self.get_parameter('cmd_vel_topic').get_parameter_value().string_value,
            callback= self.cmd_vel_callback,
            qos_profile= 1
            )
    
        self.odom_pub = self.create_publisher(
            msg_type= Odometry,
            topic= self.get_parameter('odom_topic').get_parameter_value().string_value,
            qos_profile= 10
            )
        
        # Creating the timer that will 
        timer_period = (1.0 / self.get_parameter('odom_pub_rate_hz').get_parameter_value().double_value)
        self.timer = self.create_timer(timer_period , self.generate_odom_and_tf)
        self.rel_time = self.get_clock().now().nanoseconds 


        # Creating the Transform Broadcaster that will publish the transform (odom => base_link)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.left_roboteq.connect_serial()
        self.right_roboteq.connect_serial()

        # Initial Position
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.theta_rads = 0.0


    def generate_odom_and_tf(self):
        """
        Get the velocity info from roboteqs. Then calculate pose change. Add to pose and publish.
        Then broadcast the TF.
        """

        wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        track_width = self.get_parameter('track_width').get_parameter_value().double_value

        # Get all four wheel RPMS
        rpm_cmd = RT_RUNTIME_QUERIES.Read_Encoder_Motor_Speed_in_RPM
                
        # Get the average of the RPM for each side.
        # read_runtime_query returns list of string, eval(rpm) converts list elements to int from str
        rpm_query_output = self.left_roboteq.read_runtime_query(rpm_cmd) + self.right_roboteq.read_runtime_query(rpm_cmd)
        # Set the class-wide query string.

        # Want to place the delta time calculation close to the runtime query to ensue that the time and measurement are as close as possible
 
        curr_time = int(self.get_clock().now().nanoseconds)
        delta_time = int(curr_time - self.rel_time)/ 1e9

        try:
            rpm_values = list(map(int,rpm_query_output))
            rpm_values = [ (rpm_value / DEFAULT_GEAR_REDUCTION_RATIO )for rpm_value in rpm_values ]

        except Exception as exception:
            rpm_values = [0,0,0,0]

        left_rpms = rpm_values[0:1]
        right_rpms = rpm_values[2:3]

        omega_l_avg = sum(left_rpms)/len(left_rpms)
        omega_r_avg = sum(right_rpms)/len(right_rpms)

        # Get the translational velocities for each side (m/s)
        V_r = omega_r_avg * 2 * math.pi * wheel_radius / 60 # m/s
        V_l = omega_l_avg * 2 * math.pi * wheel_radius / 60 # m/s
 
        # Get the total linear velocity of the robot body using both sides.
        V_avg = (V_r + V_l) / 2 # m / s 

        delta_theta_rads = (((V_r - V_l) / track_width) * delta_time)

        self.theta_rads += delta_theta_rads

        # Get the components of translational velocity using our current theta.
        V_x = V_avg * math.cos(self.theta_rads)
        V_y = V_avg * math.sin(self.theta_rads)

        # Now find our pose change using the velocities
        X_delta = V_x * delta_time
        Y_delta = V_y * delta_time

        # Now update our position.
        self.x_pos += X_delta
        self.y_pos += Y_delta
        
        # update the relative time and then publish odom message
        self.rel_time = int(self.get_clock().now().nanoseconds)

        odom_message = Odometry()
        odom_message.header.frame_id = "odom"
        odom_message.header.stamp = self.get_clock().now().to_msg()
        odom_message.child_frame_id = "base_link"
        odom_message.pose.pose.position.x = self.x_pos
        odom_message.pose.pose.position.y = self.y_pos
        odom_message.pose.pose.position.z = 0.0

        quats = self.quaternion_from_euler(0,0, self.theta_rads)
        quat_message = Quaternion()
        quat_message.x = quats[0]
        quat_message.y = quats[1]
        quat_message.z = quats[2]
        quat_message.w = quats[3]

        odom_message.pose.pose.orientation = quat_message
                
        self.odom_pub.publish(odom_message)

        transform_message = TransformStamped()
        transform_message.header.frame_id = "odom"
        transform_message.header.stamp = self.get_clock().now().to_msg()
        transform_message.child_frame_id = "base_link"
        transform_message.transform.translation.x = self.x_pos
        transform_message.transform.translation.y = self.y_pos
        transform_message.transform.translation.z = 0.0
        transform_message.transform.rotation = quat_message

        self.tf_broadcaster.sendTransform(transform_message)

        if ( self.get_parameter('debugging_state').get_parameter_value().bool_value ):
            self.get_logger().info('generate_odom_and_tf:\n' + \
                                   '    RPM Query Values: ' + str(rpm_query_output) + '\n' + \
                                   '    Adjusted RPM Values: ' + str(rpm_values) + '\n' + \
                                   '    Current Theta Value: ' + str(math.degrees(self.theta_rads)) + '\n'
                                   )
                            
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
        
        gear_ratio = self.get_parameter('gear_reduction_ratio').get_parameter_value().double_value
        max_rpm = self.get_parameter('max_rpm').get_parameter_value().double_value
        cmd_vel_delay_sec = self.get_parameter('cmd_vel_delay_sec').get_parameter_value().double_value
        track_width: float = self.get_parameter('track_width').get_parameter_value().double_value
        wheel_radius: float = self.get_parameter('wheel_radius').get_parameter_value().double_value
 

        def clamp(value, minimum, maximum):
            return max(minimum, min(value,maximum))
    
        right_speed = ( twist_msg.linear.x + twist_msg.angular.z * (track_width/2) ) # meters / second
        left_speed  = ( twist_msg.linear.x - twist_msg.angular.z * (track_width/2) )  # meters / second

        right_rpm =  max_rpm * clamp((( right_speed / (wheel_radius * 2 * math.pi)) * 60 * gear_ratio),-1,1) # m/s / rotations/m = rotations/sec * 60 = rotations/minute
        left_rpm = max_rpm * clamp((( left_speed / (wheel_radius * 2 * math.pi) ) * 60 * gear_ratio),-1,1) # m/s / rotations/m = rotations/sec * 60 = rotations/minute

        if(self.left_roboteq.is_open and self.right_roboteq.is_open):
            try:

                rpm_cmd = RT_RUNTIME_COMMANDS.Go_to_Speed_or_to_Relative_Position
                self.left_roboteq.write_runtime_command(rpm_cmd, [left_rpm, left_rpm])
                self.right_roboteq.write_runtime_command(rpm_cmd, [right_rpm, right_rpm])
            except Exception as serExcpt:
                self.get_logger().warn(str(serExcpt))


        if ( self.get_parameter('debugging_state').get_parameter_value().bool_value ):

            self.get_logger().info('\ncmd_vel_callback:\n' + 
                                   '    Recieved twist message: \n' + \
                                   '    Linear X: ' + str(twist_msg.linear.x) + '\n' + \
                                   '    Angular Z: ' + str(twist_msg.angular.z) + '\n' + \
                                   '    Calculated Left RPM: ' + str(left_rpm) + '\n' + \
                                   '    Calculated Right RPM ' + str(right_rpm) + '\n' 
                                   )
        
        time.sleep(cmd_vel_delay_sec)


def main(args=None):

    rclpy.init(args=args)

    roboteq_node = Roboteq_Node()
    rclpy.spin(roboteq_node)

    roboteq_node.left_roboteq.disconnect_serial()
    roboteq_node.right_roboteq.disconnect_serial()

    roboteq_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()