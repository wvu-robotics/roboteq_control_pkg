# generate_roboteq_runtime_constants.py
# By:Nathan Adkins 

# Roboteq User Manual: https://www.roboteq.com/docman-list/motor-controllers-documents-and-files/documentation/user-manual/272-roboteq-controllers-user-manual-v21/file
# Motor Configurations Table found on page 330
# Command, Arguments, Description

import csv

runtime_addresses = {}

# Open the csv that is based off of the configuration table in the user manual
for row in csv.reader(open("src/hardware_interfacing/roboteq_control_pkg/roboteq_control_pkg/roboteq_runtimes.csv"),delimiter=','):
    # Add a key/value to the configuration addess dictionary
    # Key is the "Description" in the User Manual
    # Value is the "Command" in the User Manual
    runtime_addresses.update({row[2] : row[0]})

# Write the python dictionary to a new file
new_file = open("src/hardware_interfacing/roboteq_control_pkg/roboteq_control_pkg/roboteq_runtime_constants.py","w")
new_file.write("RUNTIME_ADDRESSES =" + str(runtime_addresses))
new_file.close()

