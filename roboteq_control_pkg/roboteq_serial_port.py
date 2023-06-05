# roboteq_serial_port.py
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

import serial

LEFT_PORT = '/dev/left_roboteq'
RIGHT_PORT = '/dev/right_roboteq'
BAUD = 115200 
TIMEOUT = 5

MAX_MOTOR_COUNT = 3 
MAX_RUNTIME_COMMANDS_LENGTH = 3 
MAX_RUNTIME_QUERIES_LENGTH = 3
MAX_MAINTENANCE_COMMAND_LENGTH = 5

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