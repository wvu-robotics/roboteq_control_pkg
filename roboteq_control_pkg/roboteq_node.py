# roboteq_node.py
# By: Nathan Adkins 
# email: npa00003@mix.wvu.edu
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
import numpy as np
import math, rclpy, time
from tf2_ros import TransformBroadcaster

import rclpy
from rclpy.node import Node
import operator

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, PoseWithCovariance, TwistWithCovariance, Quaternion
from retailbot_interfaces.msg import RoboteqInfo

from .roboteq_serial_port import RoboteqSerialPort
from .roboteq_constants import rt_runtime_queries, rt_runtime_commands


'''
Important Note:

Right Motor 1 -> Back Right
Right Motor 2 -> Front Right
Left Motor 1 -> Front Left
Left Motor 2 -> Back Left 

'''

class Roboteq_Node(Node):

    def __init__(self):
        
        super().__init__('roboteq_control_node')

        # ROS PARAMETERS 
        self.declare_parameter('odom_topic', 'roboteq_odom')        
        self.declare_parameter('odom_pub_rate_hz', 100.0)

        self.declare_parameter('roboteq_info_topic', 'roboteq_info')
        self.declare_parameter('roboteq_info_pub_rate_hz', 100.0 )

        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('cmd_vel_delay_sec', 0.005 )

        self.declare_parameter('odom_frame_id', 'odom' )
        self.declare_parameter('odom_child_frame_id','base_link')

        # DEVICE, MOTOR, AND INTRINSIC PARAMETERS
        self.declare_parameter('left_roboteq_port', '/dev/left_roboteq')
        self.declare_parameter('right_roboteq_port', '/dev/right_roboteq')

        fudge_val = 10 # No idea why this works. This is correct. Going to investigate at some point in the future. 
        self.declare_parameter('gear_reduction_ratio', 0.027 * fudge_val)
        self.declare_parameter('wheel_radius', 0.125)
        self.declare_parameter('track_width', 0.77 ) 

        # SAFETY & MISC PARAMETERS
        self.declare_parameter('covariance_data_depth', 1000)
        self.declare_parameter('debugging_state', False)
        self.declare_parameter('max_linear_speed', 0.5)

        self.runtime_queries = rt_runtime_queries()
        self.runtime_commands = rt_runtime_commands()
        self.tf_broadcaster = TransformBroadcaster(self)

        self.current_x_position = 0.0
        self.current_y_position = 0.0
        self.current_theta_in_radians = 0.0

        self.twist_data_list: list[list[int]] = [[],[],[],[],[],[]] 
        self.pose_data_list: list[list[int]] = [[],[],[],[],[],[]] 

        self.roboteq_odom_timer = self.create_timer(
            timer_period_sec = (1.0 / self.get_parameter('odom_pub_rate_hz').get_parameter_value().double_value),
            callback = self.generate_odom_and_tf
            )

        # Initializing the relative time used for velocity calculations
        self.rel_time = self.get_clock().now().nanoseconds 

        # Serial class parameters (will not change for the roboteqs, adding these constants for good practice)
        DEFAULT_BAUD = 115200 # Bits per second / pulses per second (Value acceptable by roboteqs)
        DEFAULT_TIMEOUT = 5 
        MOTORS_PER_ROBOTEQ = 2 # Motors per Roboteq 

        self.query_cmds = [
                    ("Motor Amps",self.runtime_queries.Read_Motor_Amps),
                    ("Relative Encoder Count",self.runtime_queries.Read_Encoder_Count_Relative),
                    ("Absolute Encoder Count",self.runtime_queries.Read_Encoder_Counter_Absolute),
                    ("Closed Loop Error",self.runtime_queries.Read_Closed_Loop_Error),
                    ("Encoder RPM",self.runtime_queries.Read_Encoder_Motor_Speed_in_RPM),
                    ("Sensor Errors",self.runtime_queries.Read_Sensor_Errors),
                    ("Temperature",self.runtime_queries.Read_Temperature),
                    ("Firmware ID",self.runtime_queries.Read_Firmware_ID),
                    ]

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

        self.left_roboteq.connect_serial()
        self.right_roboteq.connect_serial()
        

        self.cmd_vel_sub = self.create_subscription(
            msg_type= Twist,
            topic= self.get_parameter('cmd_vel_topic').get_parameter_value().string_value,
            callback= self.cmd_vel_callback,
            qos_profile= 1
            )
        
        self.roboteq_odom_pub = self.create_publisher(
            msg_type= Odometry,
            topic= self.get_parameter('odom_topic').get_parameter_value().string_value,
            qos_profile= 10
            )
        
        self.roboteq_info_pub = self.create_publisher(
            msg_type= RoboteqInfo,
            topic= self.get_parameter('roboteq_info_topic').get_parameter_value().string_value,
            qos_profile= 10
            )
        
    def cmd_vel_callback(self, twist_msg: Twist):
        
        def linear_vel(motor_side: str):
            if motor_side.upper() == "RIGHT":
                op = operator.add
            elif motor_side.upper() == "LEFT":
                op = operator.sub
            else:
                raise ValueError
            return op(twist_msg.linear.x, twist_msg.angular.z * (self.get_parameter('track_width').get_parameter_value().double_value/2))

        def calc_rpm(lin_vel): # m/s / rotations/m = rotations/sec * 60 = rotations/minute
            gear_ratio: float = self.get_parameter('gear_reduction_ratio').get_parameter_value().double_value
            wheel_radius: float = self.get_parameter('wheel_radius').get_parameter_value().double_value
            max_rpm = (self.get_parameter('max_linear_speed').get_parameter_value().double_value) * (60.0/1.0) / (2.0 * math.pi * wheel_radius) 

            def clamp(value, minimum, maximum):
                return max(minimum, min(value,maximum))
            
            # (m/sec) / (rotations/m) = (rotations/sec) * (sec/minute) = (rotations/minute)
            return max_rpm * clamp((( lin_vel / (wheel_radius * 2 * math.pi)) * 60 * gear_ratio),-1,1)

        right_rpm =  calc_rpm(linear_vel('right')) 
        left_rpm = calc_rpm(linear_vel('left')) 

        if(self.left_roboteq.is_open and self.right_roboteq.is_open):
            try:
                rpm_cmd = self.runtime_commands.Go_to_Speed_or_to_Relative_Position
                self.left_roboteq.write_runtime_command(rpm_cmd, [left_rpm, left_rpm])
                self.right_roboteq.write_runtime_command(rpm_cmd, [right_rpm, right_rpm])
            except Exception as serExcpt:
                self.get_logger().warn(str(serExcpt))

        time.sleep(self.get_parameter('cmd_vel_delay_sec').get_parameter_value().double_value)

        if ( self.get_parameter('debugging_state').get_parameter_value().bool_value ):
            self.get_logger().info(f"\ncmd_vel_callback:\n      Calculated Left RPM: {left_rpm}\n       Calculated Right RPM: {left_rpm}\n")
        

    def generate_odom_and_tf(self):

        wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        track_width = self.get_parameter('track_width').get_parameter_value().double_value

        curr_roboteq_data = RoboteqInfo()
        curr_roboteq_data.header.stamp = self.get_clock().now().to_msg()
        curr_roboteq_data.header.frame_id = self.get_parameter("odom_frame_id").get_parameter_value().string_value

        curr_roboteq_data.front_left_motor_data = [] 
        curr_roboteq_data.back_left_motor_data = [] 
        curr_roboteq_data.front_right_motor_data = [] 
        curr_roboteq_data.back_right_motor_data = []

        curr_roboteq_data.data_description = [] 

        for cmd_str, cmd in self.query_cmds:

            # Left 1, Left 2, Right 1, Right 2
            # Front Left, Back Left, Back Right, Front Right
            serial_output = self.left_roboteq.read_runtime_query(cmd) + self.right_roboteq.read_runtime_query(cmd)
            self.get_logger().info(f"{cmd_str}, {serial_output}")

            curr_roboteq_data.data_description.append(cmd_str)
            curr_roboteq_data.front_left_motor_data.append(serial_output[0])
            curr_roboteq_data.back_left_motor_data.append(serial_output[1]) 
            curr_roboteq_data.back_right_motor_data.append(serial_output[2])
            curr_roboteq_data.front_right_motor_data.append(serial_output[3]) 

            
        self.roboteq_info_pub.publish(curr_roboteq_data)
        # Get all four wheel RPMS
        rpm_cmd = self.runtime_queries.Read_Encoder_Motor_Speed_in_RPM
        rpm_query_output = self.left_roboteq.read_runtime_query(rpm_cmd) + self.right_roboteq.read_runtime_query(rpm_cmd)


        # **Note**:
        #   This layout assumes that for the time of the calculation, retailbot is traveling at the 
        #   same RPM values as was read at the beginning of the period corresponding to delta_time
        curr_time = int(self.get_clock().now().nanoseconds)
        curr_delta_time = int(curr_time - self.rel_time)/ 1e9
        self.rel_time = curr_time

        try:
            valid_rpm_value_list: list[int] = list(map(int,rpm_query_output))
            gear_reduced_valid_rpm_values = [ (valid_rpm_value / self.get_parameter('gear_reduction_ratio').get_parameter_value().double_value ) for valid_rpm_value in valid_rpm_value_list ]

        except Exception as exception:
            self.get_logger().warn(str(exception))
            self.get_logger().warn("generate_odom_and_tf:\n     rpm_query_output: " + str(rpm_query_output) + "\n     could not be mapped to integer values, making them zeros")
            gear_reduced_valid_rpm_values = [0,0,0,0]

        clean_left_rpms = gear_reduced_valid_rpm_values[0:1]
        clean_right_rpms = gear_reduced_valid_rpm_values[2:3]

        left_rpms = sum(clean_left_rpms)/len(clean_left_rpms)
        right_rpms = sum(clean_right_rpms)/len(clean_right_rpms)

        # Get the translational velocities for each side (m/s)
        right_linear_velcity = left_rpms * 2 * math.pi * wheel_radius / 60 # m/s
        left_linear_velocity = right_rpms * 2 * math.pi * wheel_radius / 60 # m/s
 
        # Get the total linear velocity of the robot body using both sides.
        no_component_velocity = (right_linear_velcity + left_linear_velocity) / 2 

        # Update theta value by integrating the angular velocity (RADIANS)
        curr_angular_z_velocity = ((left_linear_velocity - right_linear_velcity) / track_width)
        self.current_theta_in_radians += curr_angular_z_velocity * curr_delta_time

        # Get the components of translational velocity using the current theta.
        x_velocity = no_component_velocity * math.cos(self.current_theta_in_radians)
        y_velocity = no_component_velocity * math.sin(self.current_theta_in_radians)

        # Now update the position using integrated velocity
        self.current_x_position += x_velocity * curr_delta_time
        self.current_y_position += y_velocity * curr_delta_time
        

        # Making the Odometry Message 
        # http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
        odometry_message = Odometry()
        odometry_message.header.stamp = self.get_clock().now().to_msg()
        odometry_message.header.frame_id = self.get_parameter("odom_frame_id").get_parameter_value().string_value
        odometry_message.child_frame_id = self.get_parameter("odom_child_frame_id").get_parameter_value().string_value

        # --------------------------------------------------
        # Pose with Covariance (Positions)
        # http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovariance.html
        odometry_pose_message = PoseWithCovariance()
        odometry_pose_message.pose.position.x = self.current_x_position
        odometry_pose_message.pose.position.y = self.current_y_position
        odometry_pose_message.pose.position.z = 0.0
        odometry_pose_message.pose.orientation = self.quaternion_from_euler(0,0, self.current_theta_in_radians)

        # X position, Y position, Z position, X rotation (roll), Y rotation (pitch), Z rotation (yaw)
        values_to_add_to_pose_data = [self.current_x_position, self.current_y_position, 0.0, 0.0, 0.0, self.current_theta_in_radians]

        # Updating the data used in the covariance calculations with the most up to date measurements 
        for new_value_index in range(len(values_to_add_to_pose_data)):
            self.pose_data_list[new_value_index].append(values_to_add_to_pose_data[new_value_index])

        # If the data set used to calculate the covariance is too large, delete the first element before adding the new values on
        if ( len(self.pose_data_list[0]) >= self.get_parameter('covariance_data_depth').get_parameter_value().integer_value ):
            # X Velocity, Y Velocity, Z Velocity, Angular X Velocity, Angular Z Velocity, Angular Z Velocity
            for twist_component_data_set in self.pose_data_list:
                twist_component_data_set.pop(0)

        # CALCULATING COVARIANCE
        # https://datatofish.com/covariance-matrix-python/
        pose_data = np.array(self.pose_data_list)
        odometry_pose_message.covariance = np.cov(pose_data, bias=True).flatten()

        # Populating the odometry message with the pose message that was just created 
        odometry_message.pose = odometry_pose_message
        # Pose with Covariance (Positions)
        # --------------------------------------------------


        # --------------------------------------------------
        # Twist with Covariance (Velocities)
        # http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistWithCovariance.html
        odometry_twist_message = TwistWithCovariance()
        odometry_twist_message.twist.linear.x = x_velocity
        odometry_twist_message.twist.linear.y = y_velocity
        odometry_twist_message.twist.linear.z = 0.0
        odometry_twist_message.twist.angular.x = 0.0
        odometry_twist_message.twist.angular.y = 0.0
        odometry_twist_message.twist.angular.z = curr_angular_z_velocity

        # X Velocity, Y Velocity, Z Velocity, Angular X Velocity, Angular Z Velocity, Angular Z Velocity
        values_to_add_to_twist_data = [x_velocity, y_velocity, 0.0, 0.0, 0.0, curr_angular_z_velocity]

        # Updating the data used in the covariance calculations with the most up to date measurements 
        for new_value_index in range(len(values_to_add_to_twist_data)):
            self.twist_data_list[new_value_index].append(values_to_add_to_pose_data[new_value_index])

        # If the data set used to calculate the covariance is too large, delete the first element before adding the new values on
        if ( len(self.twist_data_list[0]) >= self.get_parameter('covariance_data_depth').get_parameter_value().integer_value ):
            # X Velocity, Y Velocity, Z Velocity, Angular X Velocity, Angular Z Velocity, Angular Z Velocity
            for twist_component_data_set in self.twist_data_list:
                twist_component_data_set.pop(0)

        # CALCULATING COVARIANCE
        # https://datatofish.com/covariance-matrix-python/
        twist_data = np.array(self.twist_data_list)
        odometry_twist_message.covariance = np.cov(twist_data, bias=True).flatten()

        # Populating the odometry message with the twist message that was just created 
        odometry_message.twist = odometry_twist_message

        # Twist with Covariance (Velocities)
        # --------------------------------------------------

        # Making the Transform Message
        # http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TransformStamped.html
        transform_message = TransformStamped()
        transform_message.header.frame_id = self.get_parameter("odom_frame_id").get_parameter_value().string_value
        transform_message.header.stamp = self.get_clock().now().to_msg()
        transform_message.child_frame_id = self.get_parameter("odom_child_frame_id").get_parameter_value().string_value
        transform_message.transform.translation.x = self.current_x_position
        transform_message.transform.translation.y = self.current_y_position
        transform_message.transform.translation.z = 0.0
        transform_message.transform.rotation = self.quaternion_from_euler(0,0, self.current_theta_in_radians)

        # Publishing the Odometry Message 
        self.roboteq_odom_pub.publish(odometry_message)

        # Sending the Transform Message
        self.tf_broadcaster.sendTransform(transform_message)

        if ( self.get_parameter('debugging_state').get_parameter_value().bool_value ):
            self.get_logger().info('generate_odom_and_tf:\n' + \
                                   '    RPM Query Values: ' + str(rpm_query_output) + '\n' + \
                                   '    Adjusted RPM Values (Rounded): ' + str([int(rpm_value) for rpm_value in gear_reduced_valid_rpm_values]) + '\n' + \
                                   '    Current Theta Value (Rounded): ' + str(int(math.degrees(self.current_theta_in_radians))) + '\n' + \
                                   '    Current Position: ' + str([self.current_x_position,self.current_y_position]) + '\n' \

                                   'curr_angular_z_velocity: ' + str(curr_angular_z_velocity)
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
        quaternion_message = Quaternion()
        quaternion_message.x = cj*sc - sj*cs
        quaternion_message.y = cj*ss + sj*cc
        quaternion_message.z = cj*cs - sj*sc
        quaternion_message.w = cj*cc + sj*ss
        return quaternion_message


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