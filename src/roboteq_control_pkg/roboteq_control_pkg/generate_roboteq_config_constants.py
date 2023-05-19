# generate_roboteq_config_constants.py
# By:Nathan Adkins 

# Roboteq User Manual: https://www.roboteq.com/docman-list/motor-controllers-documents-and-files/documentation/user-manual/272-roboteq-controllers-user-manual-v21/file
# Motor Configurations Table found on page 330
# Command, Arguments, Description

import csv

configuration_addresses = {}

# Open the csv that is based off of the configuration table in the user manual
for row in csv.reader(open("src/hardware_interfacing/roboteq_control_pkg/roboteq_control_pkg/roboteq_configurations.csv"),delimiter=','):
    # Add a key/value to the configuration addess dictionary
    # Key is the "Description" in the User Manual
    # Value is the "Command" in the User Manual
    configuration_addresses.update({row[2] : row[0]})

# Write the python dictionary to a new file
new_file = open("src/hardware_interfacing/roboteq_control_pkg/roboteq_control_pkg/roboteq_config_constants.py","w")
new_file.write("CONFIGURATION_ADDRESSES =" + str(configuration_addresses))
new_file.close()

