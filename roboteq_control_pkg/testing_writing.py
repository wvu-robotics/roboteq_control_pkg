import serial, time
from roboteq_constants import MOTOR_CONFIG, RUNTIME_COMMANDS, RUNTIME_QUERIES

LEFT_PORT = '/dev/left_roboteq'

BAUD = 115200 
TIMEOUT = 5

MAX_MOTOR_COUNT = 3 
MAX_RUNTIME_COMMANDS_LENGTH = 3 
MAX_RUNTIME_QUERIES_LENGTH = 3
MAX_MAINTENANCE_COMMAND_LENGTH = 5

speed_cmd = RUNTIME_COMMANDS.get('Go to Speed or to Relative Position')
set_acc = RUNTIME_COMMANDS.get('Set Acceleration')
motor_speed = RUNTIME_QUERIES.get('Read BL Motor Speed in RPM')


class RoboteqSerialPort(serial.Serial):


    def __init__(self, port, baudrate, timeout, num_motors_connected):
        super(RoboteqSerialPort,self).__init__(
            port= port,
            baudrate= baudrate,
            timeout= timeout,
        )
        if num_motors_connected > MAX_MOTOR_COUNT:
            Exception("Invalid number of motors for Roboteq")
        self.motor_count = num_motors_connected


    def write_runtime_command(self, cmd_str: str, cmd_vals: list[str]):
        runtime_char = '!'
        if len(cmd_str) > MAX_RUNTIME_COMMANDS_LENGTH:
            Exception("Invalid command length for runtime commands.")
        motor_cmd_string = ''
        for motor_num in range(self.motor_count):
            motor_cmd_string += f'{runtime_char}{cmd_str} {motor_num+1} {cmd_vals[motor_num]}\r'
        self.write(motor_cmd_string.encode())


    def write_runtime_query(self, cmd_str: str):
        query_char = '?'
        if len(cmd_str) > MAX_RUNTIME_QUERIES_LENGTH:
            Exception("Invalid command length for runtime queries.")
        motor_cmd_string = ''
        for motor_num in range(self.motor_count):
            motor_cmd_string += f'{query_char}{cmd_str} {motor_num+1}\r'
        print( self.write(motor_cmd_string.encode()))


    def write_maintenance_command(self, cmd_str: str):
        maint_char = '%'
        safety_key = 321654987
        if len(cmd_str) > MAX_MAINTENANCE_COMMAND_LENGTH:
            Exception("Invalid command length for maintenance commands.")
        motor_cmd_string = ''
        for motor_num in range(self.motor_count):
            motor_cmd_string += f'{maint_char}{cmd_str} {motor_num+1} \r'
        
        print(self.read_until(b"r"))


help_me = RoboteqSerialPort(LEFT_PORT,BAUD,TIMEOUT,2)


help_me.write_runtime_command(speed_cmd, [-5000,-5000])
time.sleep(1)
help_me.write_runtime_query(motor_speed)

# help_me.write_runtime_command(set_acc, 50)
# help_me.write_runtime_command(speed_cmd, 0)
# help_me.write_runtime_command(speed_cmd, 5000)
# time.sleep(1)

# help_me.write_runtime_command(set_acc, 500)
# time.sleep(1)

# help_me.write_runtime_command(speed_cmd, 0)
# help_me.write_runtime_command(speed_cmd, 5000)


