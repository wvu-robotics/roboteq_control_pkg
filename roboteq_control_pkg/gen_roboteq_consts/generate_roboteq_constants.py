# generate_roboteq_constants.py
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

import csv, os

GENERATED_FILE_PATH = 'src/hardware_interfacing/roboteq_control_pkg/roboteq_control_pkg/roboteq_constants.py'
CSV_DIRECTORY = 'src/hardware_interfacing/roboteq_control_pkg/roboteq_control_pkg/gen_roboteq_consts'
REPLACED_CHARACTERS = [' ','&','-','/','(',')']


def generate_constants_from_csv():
    csv_files: str = []
    const_names: str = []

    for curr_csv_file_name in os.listdir(CSV_DIRECTORY):

        curr_csv_file = os.path.join(CSV_DIRECTORY, curr_csv_file_name)

        if os.path.isfile(curr_csv_file) and curr_csv_file.endswith('.csv'): 
            
            const_names.append(curr_csv_file_name.replace('.csv',''))
            csv_files.append(curr_csv_file)

            # Open the new constants file in write mode
            new_file = open(GENERATED_FILE_PATH, "w")
            new_file.write("# This file was autogenerated by:\n#     /roboteq_control_pkg/roboteq_control_pkg/gen_roboteq_consts/generate_roboteq_constants.py\n\n")
     

            # Iterating through the csv files 
            for curr_const_index in range(len(csv_files)): 

                # Defining class header and class instance names for the string being written to the new file
                class_header = const_names[curr_const_index].lower()
                class_instance_name = const_names[curr_const_index].upper()

                # Opening current csv file
                curr_file = open(csv_files[curr_const_index]) 

                # Creating code for new class and new class constructor in the string being written to the new file
                curr_class_str = 'class _' + class_header + "():\n" 
                curr_class_str += '\n  def __init__(self): \n'


                # Iterating through the contents of the current csv file 
                for row in csv.reader(curr_file, delimiter=','):

                    var_name = row[2]

                    # Check for invalid characters before creating the class attributes in the string being written to the new file
                    for curr_replaced_char in REPLACED_CHARACTERS:
                        var_name = var_name.replace(curr_replaced_char, '_')

                    # Create a class attribute with the description of the command, and assign command string to it in the string being written to the new file
                    curr_class_str += '      self.' + var_name + ' = ' + '\"' + row[0] + '\"\n' 
                

                # Create an instance of the class in the string being written to the new file
                curr_class_str += '\n' + class_instance_name + ' = _' + class_header + '()\n\n'
                new_file.write(curr_class_str + "\n")

    print("Created the constants file at the following file path:\n" + "     " + GENERATED_FILE_PATH)


def main():
    generate_constants_from_csv()


if __name__ == '__main__':
    main()