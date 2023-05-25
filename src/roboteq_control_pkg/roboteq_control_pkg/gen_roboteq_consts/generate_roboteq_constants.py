# generate_roboteq_constants.py
# By:Nathan Adkins 

# Roboteq User Manual: https://www.roboteq.com/docman-list/motor-controllers-documents-and-files/documentation/user-manual/272-roboteq-controllers-user-manual-v21/file
# Motor Configurations Table found on page 330
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
        new_file.write(dict_names[curr_dict_index] + " = " + str(curr_dict) + "\n")

        # Clear the memory address to eliminate any weird python dictionary behaviors
        del curr_dict 


generate_dict_from_csv(

    new_file_name= "src/hardware_interfacing/roboteq_control_pkg/src/roboteq_control_pkg/roboteq_control_pkg/roboteq_constants.py",

    csv_files= 
    ["src/hardware_interfacing/roboteq_control_pkg/src/roboteq_control_pkg/roboteq_control_pkg/gen_roboteq_consts/roboteq_config_constants.csv",
     "src/hardware_interfacing/roboteq_control_pkg/src/roboteq_control_pkg/roboteq_control_pkg/gen_roboteq_consts/roboteq_runtime_constants.csv",
    ],

    dict_names= 
    ["CONFIG_CONSTS", 
    "RUNTIME_CONSTS",
    ],
)
