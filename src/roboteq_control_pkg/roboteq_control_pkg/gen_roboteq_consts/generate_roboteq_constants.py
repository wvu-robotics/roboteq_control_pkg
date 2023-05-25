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

# CSV format:
# Command, Arguments, Description

import csv

def generate_dict_from_csv(new_file_name: str, csv_files: list[str], dict_names: list[str]):

    # Input validation: Checking to see if the lengths of the lists are the same
    if len(csv_files) != len(dict_names):
        raise Exception("The number of files and dictionary names is not the same.")
    
    # Open the new constants file in write mode
    # No need for FileNotFoundError, python will create file where you tell it, or overwrite existing contents
    new_file = open(new_file_name, "w")
    
    # Iterating through the csv files 
    for curr_dict_index in range(len(csv_files)):
        curr_dict = {}

        try:
            # Opening current csv file
            curr_file = open(csv_files[curr_dict_index])

            # For each row in current csv file, assign "description" to "command" in the current dictionary
            for row in csv.reader(curr_file, delimiter=','):
                curr_dict.update({row[2] : row[0]})

        except FileNotFoundError as file_error:
            print("The file at " + csv_files[curr_dict_index] + " was not found.")
            print(str(file_error))

        except Exception as error:
            print(str(error))

        # Write the dictionary to the new constants file with the dictionary name and the current dictionary's value
        dict_contents = dict_names[curr_dict_index] + " = " + str(curr_dict)

        # Place newline characters after each dictionary key/value pair
        dict_contents = dict_contents.replace(",",", \n")
        # Make curly brakets have their own line (start of dictionary)
        dict_contents = dict_contents.replace("{","{\n")
        # Make curly brakets have their own line (end of dictionary)
        dict_contents = dict_contents.replace("}","\n}")

        # Write the dictionary to the new constants file
        new_file.write(dict_contents + "\n")
        
        # Clear the memory address to eliminate any weird python dictionary behaviors
        del curr_dict 

generate_dict_from_csv(

    new_file_name= "src/hardware_interfacing/roboteq_control_pkg/src/roboteq_control_pkg/roboteq_control_pkg/roboteq_constants.py",

    csv_files= 
        [
        "src/hardware_interfacing/roboteq_control_pkg/src/roboteq_control_pkg/roboteq_control_pkg/gen_roboteq_consts/rt_ac_induction_specific_commands.csv",
        "src/hardware_interfacing/roboteq_control_pkg/src/roboteq_control_pkg/roboteq_control_pkg/gen_roboteq_consts/rt_analog_digital_pluse_io_configurations.csv",
        "src/hardware_interfacing/roboteq_control_pkg/src/roboteq_control_pkg/roboteq_control_pkg/gen_roboteq_consts/rt_brushless_specific_commands.csv",
        "src/hardware_interfacing/roboteq_control_pkg/src/roboteq_control_pkg/roboteq_control_pkg/gen_roboteq_consts/rt_can_communication_commands.csv",
        "src/hardware_interfacing/roboteq_control_pkg/src/roboteq_control_pkg/roboteq_control_pkg/gen_roboteq_consts/rt_ds402_runtime_commands.csv",
        "src/hardware_interfacing/roboteq_control_pkg/src/roboteq_control_pkg/roboteq_control_pkg/gen_roboteq_consts/rt_ds402_runtime_queries.csv",
        "src/hardware_interfacing/roboteq_control_pkg/src/roboteq_control_pkg/roboteq_control_pkg/gen_roboteq_consts/rt_general_configuration_and_safety.csv",
        "src/hardware_interfacing/roboteq_control_pkg/src/roboteq_control_pkg/roboteq_control_pkg/gen_roboteq_consts/rt_maintenance_commands.csv",
        "src/hardware_interfacing/roboteq_control_pkg/src/roboteq_control_pkg/roboteq_control_pkg/gen_roboteq_consts/rt_motor_configurations.csv",
        "src/hardware_interfacing/roboteq_control_pkg/src/roboteq_control_pkg/roboteq_control_pkg/gen_roboteq_consts/rt_runtime_commands.csv",
        "src/hardware_interfacing/roboteq_control_pkg/src/roboteq_control_pkg/roboteq_control_pkg/gen_roboteq_consts/rt_runtime_queries.csv",
        "src/hardware_interfacing/roboteq_control_pkg/src/roboteq_control_pkg/roboteq_control_pkg/gen_roboteq_consts/tcp_communication_commands.csv",
        ],

    dict_names= 
    ["AC_INDUCTION_SPECIFIC_COMMANDS",
     "ANALOG_DIGITAL_PULSE_CONFIG",
     "BRUSHLESS_SPECIFIC_COMMANDS",
     "CAN_COMMANDS",
     "DS402_RUNTIME_COMMANDS",
     "DS402_RUNTIME_QUERIES",
     "GENERAL_CONFIG_SAFETY",
     "MAINTENANCE_COMMANDS",
     "MOTOR_CONFIG",
     "RUNTIME_COMMANDS",
     "RUNTIME_QUERIES",
     "TCP_COMMANDS",
    ],
)
