# Author: Ryan Sandzimier
# KAMATSU Project
# Run this script when USB devices are unplugged and plugged back in to automatically detect dev path and write to proper launch files

import commands
import os

# Get list of serial devices by ID including the dev path
output_serial_by_id = commands.getstatusoutput('ls -l /dev/serial/by-id')[1].splitlines()

# ID's for each device
id_dynamixel = 'usb-FTDI_FT232R_USB_UART_AI03QD8O-if00-port0'
id_IMU1 = 'usb-FTDI_FT232R_USB_UART_A903AAUQ-if00-port0'
id_IMU2 = 'usb-FTDI_FT232R_USB_UART_A903AAV3-if00-port0'
id_arduino = 'usb-Arduino_Srl_Arduino_Mega_7563331313335160C0C1-if00'
id_optoforce = 'usb-OptoForce_OptoForce_DAQ-if00'

key_str = '../../' # In this list, port name is always preceded by the string '../../' Use this when parsing list to know where start of port name is

dev_path_dynamixel = ''
dev_path_IMU1 = ''
dev_path_IMU2 = ''
dev_path_arduino = ''
dev_path_optoforce = ''

dev_dir = '/dev/' # dev path always starts with '/dev/'

# Loop through each line of list. If line has the key ('../../') and the device ID, store the dev path for that device ID. 
for i in range(0, len(output_serial_by_id)):
    index_key_str = output_serial_by_id[i].find(key_str)
    if index_key_str == -1: 
        continue
    if output_serial_by_id[i].find(id_dynamixel) != -1:
        dev_path_dynamixel = dev_dir + output_serial_by_id[i][index_key_str+len(key_str):]
        print 'DYNAMIXEL: ' + dev_path_dynamixel
    elif output_serial_by_id[i].find(id_IMU1) != -1:
        dev_path_IMU1 = dev_dir + output_serial_by_id[i][index_key_str+len(key_str):]
        print 'IMU1: ' + dev_path_IMU1
    elif output_serial_by_id[i].find(id_IMU2) != -1:
        dev_path_IMU2 = dev_dir + output_serial_by_id[i][index_key_str+len(key_str):]
        print 'IMU2: ' + dev_path_IMU2
    elif output_serial_by_id[i].find(id_arduino) != -1:
        dev_path_arduino = dev_dir + output_serial_by_id[i][index_key_str+len(key_str):]
        print 'ARDUINO: ' + dev_path_arduino
    elif output_serial_by_id[i].find(id_optoforce) != -1:
        dev_path_optoforce = dev_dir + output_serial_by_id[i][index_key_str+len(key_str):]
        print 'OPTOFORCE: ' + dev_path_optoforce

# Check that all device dev paths were found in the list. Exit if any were not found.
found_all = True

if dev_path_dynamixel == '':
    found_all = False
    print 'DYNAMIXEL DEV PATH NOT FOUND'
if dev_path_IMU1 == '':
    found_all = False
    print 'IMU1 DEV PATH NOT FOUND'
if dev_path_IMU2 == '':
    found_all = False
    print 'IMU2 DEV PATH NOT FOUND'
if dev_path_arduino == '':
    found_all = False
    print 'ARDUINO DEV PATH NOT FOUND'
if dev_path_optoforce == '':
    found_all = False
    print 'OPTOFORCE DEV PATH NOT FOUND'

if not found_all:
    print 'ONE OR MORE DEVICE DEV PATHS NOT FOUND. EXITING WITHOUT WRITING TO LAUNCH FILES'
    exit()

# File names to update. File path assumes starting from main directory ros_ws/
file_name1 = 'src/exp_excavator/launch/run_arduino_joystick.launch'
file_name2 = 'src/optoforce/launch/optoforce.launch'

# Check that current directory is inside main directory ros_ws. If not, exit.
current_directory = os.getcwd()
main_dir = 'ros_ws'
index = current_directory.find(main_dir)
if index == -1:
    print 'UNABLE TO FIND MAIN DIRECTORY ' + main_dir + '. EXITING WITHOUT WRITING TO LAUNCH FILES. RUN SCRIPT FROM WITHIN THE MAIN DIRECTORY ' + main_dir
    exit()

# Add '../' prior to file names to "navigate backwards" to main directory ros_ws (assumed starting directory)
index += len(main_dir)
while index < len(current_directory) and index != -1:
    index = current_directory.find('/',index+1,len(current_directory))
    file_name1 = '../' + file_name1
    file_name2 = '../' + file_name2

# Open files and store each line
f = open(file_name1,'r')
file_text1 = f.read().splitlines(1)
f.close()

f = open(file_name2,'r')
file_text2 = f.read().splitlines(1)
f.close()

# Initialize 'key' parameters for each device. The 'keys' are used wen parsing the launch files to determine the line and text within the line that must be overwritten

dynamixel_key1 = 'name="dynamixel_pro_controller"'
dynamixel_key2 = '<rosparam>'
dynamixel_key3 = "device: '"

IMU1_key1 = 'name="IMU1"'
IMU1_key2 = 'name="port"'
IMU1_key3 = 'value="'

IMU2_key1 = 'name="IMU2"'
IMU2_key2 = 'name="port"'
IMU2_key3 = 'value="'

serialnode_key1 = 'name="serial_node"'
serialnode_key2 = 'name="port"'
serialnode_key3 = 'value="'

optoforce_key1 = '<arg name="port"'
optoforce_key2 = 'default="'

dynamixel_key1_found = False
dynamixel_key2_found = False
dynamixel_key3_found = False
IMU1_key1_found = False
IMU1_key2_found = False
IMU1_key3_found = False
IMU2_key1_found = False
IMU2_key2_found = False
IMU2_key3_found = False
serialnode_key1_found = False
serialnode_key2_found = False
serialnode_key3_found = False
optoforce_key1_found = False
optoforce_key2_found = False

# Loop through each line in run_arduino_joystick.launch, looking for 'keys'. 'Keys' must be found in the proper order. When line that must be overwritten is found, store new line (to be written to launch file later). 
i = 0
while i < len(file_text1):
    # Conditions for finding Dynamixel 'keys' in order. Once line of interest is found, store new line to be written to file later
    if not dynamixel_key1_found and file_text1[i].find(dynamixel_key1) != -1:
        dynamixel_key1_found = True
    elif dynamixel_key1_found and not dynamixel_key2_found and file_text1[i].find(dynamixel_key2) != -1: 
        dynamixel_key2_found = True
    elif dynamixel_key1_found and dynamixel_key2_found and not dynamixel_key3_found and file_text1[i].find(dynamixel_key3) != -1:
        dynamixel_key3_found = True
        index1 = file_text1[i].find(dynamixel_key3)+len(dynamixel_key3)
        index2 = file_text1[i].find("'",index1,len(file_text1[i]))
        file_text1[i] = file_text1[i][:index1] + dev_path_dynamixel + file_text1[i][index2:]
    # Conditions for finding IMU1 'keys' in order. Once line of interest is found, store new line to be written to file later
    elif not IMU1_key1_found and file_text1[i].find(IMU1_key1) != -1:
        IMU1_key1_found = True
    elif IMU1_key1_found and not IMU1_key2_found and file_text1[i].find(IMU1_key2) != -1:
        index1 = file_text1[i].find(IMU1_key2)+len(IMU1_key2)
        if file_text1[i].find(IMU1_key3, index1, len(file_text1[i])) != -1:
            IMU1_key2_found = True
            IMU1_key3_found = True
            index2 = file_text1[i].find(IMU1_key3, index1, len(file_text1[i]))+len(IMU1_key3)
            index3 = file_text1[i].find('"',index2,len(file_text1[i]))
            file_text1[i] = file_text1[i][:index2] + dev_path_IMU1 + file_text1[i][index3:]
    # Conditions for finding IMU2 'keys' in order. Once line of interest is found, store new line to be written to file later
    elif not IMU2_key1_found and file_text1[i].find(IMU2_key1) != -1:
        IMU2_key1_found = True
    elif IMU2_key1_found and not IMU2_key2_found and file_text1[i].find(IMU2_key2) != -1:
        index1 = file_text1[i].find(IMU2_key2)+len(IMU2_key2)
        if file_text1[i].find(IMU2_key3, index1, len(file_text1[i])) != -1:
            IMU2_key2_found = True
            IMU2_key3_found = True
            index2 = file_text1[i].find(IMU2_key3, index1, len(file_text1[i]))+len(IMU2_key3)
            index3 = file_text1[i].find('"',index2,len(file_text1[i]))
            file_text1[i] = file_text1[i][:index2] + dev_path_IMU2 + file_text1[i][index3:]
    # Conditions for finding rosserial node 'keys' in order. Once line of interest is found, store new line to be written to file later
    elif not serialnode_key1_found and file_text1[i].find(serialnode_key1) != -1:
        serialnode_key1_found = True
    elif serialnode_key1_found and not serialnode_key2_found and file_text1[i].find(serialnode_key2) != -1:
        index1 = file_text1[i].find(serialnode_key2)+len(serialnode_key2)
        if file_text1[i].find(serialnode_key3, index1, len(file_text1[i])) != -1:
            serialnode_key2_found = True
            serialnode_key3_found = True
            index2 = file_text1[i].find(serialnode_key3, index1, len(file_text1[i]))+len(serialnode_key3)
            index3 = file_text1[i].find('"',index2,len(file_text1[i]))
            file_text1[i] = file_text1[i][:index2] + dev_path_arduino + file_text1[i][index3:]
    i += 1

# Loop through each line in optoforce.launch, looking for 'keys'. 'Keys' must be found in the proper order. When line that must be overwritten is found, store new line (to be written to launch file later). 
i=0
while i < len(file_text2):
    # Conditions for finding optoforce 'keys' in order. Once line of interest is found, store new line to be written to file later
    if not optoforce_key1_found and file_text2[i].find(optoforce_key1) != -1:
        index1 = file_text2[i].find(optoforce_key1)+len(optoforce_key1)
        if file_text2[i].find(optoforce_key2, index1, len(file_text2[i])) != -1:
            optoforce_key1_found = True
            optoforce_key2_found = True
            index2 = file_text2[i].find(optoforce_key2, index1, len(file_text2[i]))+len(optoforce_key2)
            index3 = file_text2[i].find('"',index2,len(file_text2[i]))
            file_text2[i] = file_text2[i][:index2] + dev_path_optoforce + file_text2[i][index3:]
    i += 1

# Check if successfully parsed launch files and found lines of interest (found the lines to be overwritten). If any lines of interest not found, exit without writing to any files
found_all = True

if not dynamixel_key3_found:
    found_all = False
    print 'DYNAMIXEL DEV PATH WRITE LOCATION NOT FOUND IN ' + file_name1 + ' WHILE PARSING'
if not IMU1_key3_found:
    found_all = False
    print 'IMU1 DEV PATH WRITE LOCATION NOT FOUND IN ' + file_name1 + ' WHILE PARSING'
if not IMU2_key3_found:
    found_all = False
    print 'IMU2 DEV PATH WRITE LOCATION NOT FOUND IN ' + file_name1 + ' WHILE PARSING'
if not serialnode_key3_found:
    found_all = False
    print 'SERIAL NODE DEV PATH WRITE LOCATION NOT FOUND IN ' + file_name1 + ' WHILE PARSING'
if not optoforce_key2_found:
    found_all = False
    print 'OPTOFORCE DEV PATH WRITE LOCATION NOT FOUND IN ' + file_name2 + ' WHILE PARSING'

if not found_all:
    print 'ONE OR MORE DEV PATH WRITE LOCATIONS NOT FOUND IN ' + file_name1 + ' AND ' + filename2 + '. EXITING WITHOUT WRITING TO LAUNCH FILES'
    exit()


# Write to launch files
f = open(file_name1, 'w')
for i in range(0,len(file_text1)):
    f.write(file_text1[i])
f.close()
print 'FINISHED WRITING TO FILE: ' + file_name1

f = open(file_name2, 'w')
for i in range(0,len(file_text2)):
    f.write(file_text2[i])
f.close()
print 'FINISHED WRITING TO FILE: ' + file_name2
    

                










