#!/usr/bin/env python
# coding: utf-8

# In[1]:


import PySimpleGUI as sg
import os.path
import MoCapStream
import serial
import time
from numpy import array
from numpy.linalg import norm
import numpy as np

# The calibration function. It is important to know where the centre of rotation for each servo is, so the command is
# sent to the arduino and the centre rotation is captured. The 6D rotation matrix is pulled by MoCapStream and 
# the associated rotational euler angles are extracted
def calibration(arduino, R31, R11, R21):
    arduino.write(bytes('c', 'utf-8'))   #Sends calibration command
    time.sleep(0.5)
    yaw_centre, pitch_centre = angles_from_rot(R31, R11, R21)
    return True, yaw_centre, pitch_centre

def angles_from_rot(R31, R11, R21):
    pitch_centre = -np.arcsin(R31)
    yaw_centre = np.arctan2((R21/np.cos(pitch_centre)), (R11/np.cos(pitch_centre)))
    return yaw_centre, pitch_centre

def aim_at(target, obj):
    D = np.subtract(target, obj)
    yaw_angle = np.degrees(np.arctan2(D[1], D[0]))
    pitch_angle = 90 - np.degrees(np.arctan2(np.sqrt(D[0]**2 + D[1]**2), D[2]))
    return yaw_angle, pitch_angle
    

#a rotation matrix generator generally used to test the calibration algorithm
def rotation_matrix(alpha, beta, gamma):
    # Convert angles to radians
    alpha_rad = np.radians(alpha)
    beta_rad = np.radians(beta)
    gamma_rad = np.radians(gamma)

    # Define the rotation matrices for each axis
    Rz = np.array([[np.cos(alpha_rad), -np.sin(alpha_rad), 0],
                   [np.sin(alpha_rad), np.cos(alpha_rad), 0],
                   [0, 0, 1]])

    Ry = np.array([[np.cos(beta_rad), 0, np.sin(beta_rad)],
                   [0, 1, 0],
                   [-np.sin(beta_rad), 0, np.cos(beta_rad)]])

    Rx = np.array([[1, 0, 0],
                   [0, np.cos(gamma_rad), -np.sin(gamma_rad)],
                   [0, np.sin(gamma_rad), np.cos(gamma_rad)]])

    # Combine the rotation matrices to get the final rotation matrix
    rotation_matrix = np.matmul(Rz, Ry)
    rotation_matrix = np.matmul(rotation_matrix, Rx)
    
    return rotation_matrix

def make_target_popup():
    target_popup_layout = [[sg.Text('X: '), sg.Text('Y: '), sg.Text('Z: ')],
                           [sg.Input(), sg.Input(), sg.Input()]]
    choice, target = sg.Window('Enter Target Position', [[target_popup_layout],[sg.OK(s=10), sg.Cancel(s=10)]], disable_close=True, keep_on_top=True).read(close=True)
    if choice == "Cancel" or target[0] == '' or target[1] == '' or target[2] == '':
        return choice, [np.nan, np.nan, np.nan]
    else:
        return choice, [float(target[0]), float(target[1]), float(target[2])]

default_qns_ip = "127.0.0.1"

x = 0
y = 1
z = 2
target = [np.nan, np.nan, np.nan]
servo = [-10, -10, 35]
delta_time = 0.1
arduino_connected = False
qns_connected = False
calibration_success = False

#Create a sample rotation matrix
#rotation = rotation_matrix(30, 80, 0)
rotation = [target, target, target]

sg.theme('Default1') #makes a theme to look nice


#set up the arduino frame to allow connection to any port as input by the user 
# without code needing to be changed
arduino_layout = []
arduino_layout.append(sg.Text('Arduino Port: '))
arduino_layout.append(sg.Input(key = '-PORT_ADD-'))
arduino_layout.append(sg.Button(button_text = 'Connect', enable_events=True, key = '-A_CONNECT-'))
arduino_layout = [arduino_layout,[sg.Text('Connected: '), sg.Text('False', key = '-A_CONNECTED_STATUS-')]]
arduino_frame = sg.Frame("ARDUINO", arduino_layout, s = (500, 90))

#set up the QNS to be connected with a button click
sg_qns_layout = []
#REMEMBER TO CHANGE THIS:
sg_qns_layout.append(sg.Text("QNS Port: "))
sg_qns_layout.append(sg.Input(default_text = default_qns_ip, key = '-QNS_ADD-'))
sg_qns_layout.append(sg.Button(button_text = 'Connect', enable_events=True, key = '-QNS_CONNECT-'))
sg_qns_layout = [sg_qns_layout, [sg.Text('Connected: '), sg.Text('False', key = '-QNS_CONNECTED_STATUS-')]]
qns_frame = sg.Frame("QNS", sg_qns_layout, s = (500, 90))

right_col = [[arduino_frame],[qns_frame]]

#set up the rotation output to pull and display the current rotation
sg_rotation_layout = []
sg_rotation_layout = [
                      [sg.Text(rotation[0][0], key = '-R00-', s =(18, 1)), sg.Text(rotation[0][1], key = '-R01-', s =(18, 1)), sg.Text(rotation[0][2], key = '-R02-', s =(18, 1))],
                      [sg.Text(rotation[1][0], key = '-R10-', s =(18, 1)), sg.Text(rotation[1][1], key = '-R11-', s =(18, 1)), sg.Text(rotation[1][2], key = '-R12-', s =(18, 1))],
                      [sg.Text(rotation[2][0], key = '-R20-', s =(18, 1)), sg.Text(rotation[2][1], key = '-R21-', s =(18, 1)), sg.Text(rotation[2][2], key = '-R22-', s =(18, 1))]
                      ]

rotation_frame = sg.Frame("Robot 6D Solid Body Rotation Matrix", sg_rotation_layout, key = "-ROT_TABLE-", s = (500, 100))

#set up the current position, yaw and pitch output as a frame
sg_coords_layout = [
                    [sg.Text('X: '), sg.Text(' ', key = '-O_X-', s = (15, 1)), sg.Text('Y: '), sg.Text(' ', key = '-O_Y-', s = (15, 1)), sg.Text('Z: '), sg.Text(' ', key = '-O_Z-', s = (15, 1))],
                    [sg.Text('Yaw: '), sg.Text(' ', key = '-YAW-', s = (20, 1)), sg.Text('Pitch: '), sg.Text(' ', key = '-PITCH-', s = (20, 1))]
                    ]
sg_coords_frame = sg.Frame("Current Servo Position and Rotational Angles", sg_coords_layout, key= '-COORDS-', s = (500, 80))

#set up the target position and target yaw + pitch as a frame with buttons associated
target_menu = ['Targetting',['Via QNS', 'Manual Input']]
sg_target_layout = [
                    [sg.Text('X: '), sg.Text(target[x], key = '-T_X-'), sg.Text('Y: '), sg.Text(target[y], key = '-T_Y-'), sg.Text('Z: '), sg.Text(target[z], key = '-T_Z-')],
                    [sg.Text('Angles From Zero:')],
                    [sg.Text('Desired Yaw: '), sg.Text(' ', key = '-D_YAW_FROM_ZERO-'), sg.Text('Desired Pitch: '), sg.Text(' ', key = '-D_PITCH_FROM_ZERO-')],
                    [sg.Text('Angles From Minimum Servo Angle:')],
                    [sg.Text('Desired Yaw: '), sg.Text(' ', key = '-D_YAW_FROM_MIN-'), sg.Text('Desired Pitch: '), sg.Text(' ', key = '-D_PITCH_FROM_MIN-')],
                    [sg.Text('Yaw Duty Cycle (ms): '), sg.Text(' ', key = '-YAW_DUTY-'), sg.Text('Pitch Duty Cycle (ms): '), sg.Text(' ', key = '-PITCH_DUTY-')],
                    [sg.Button('Manual Input', key = '-MANUAL_INPUT-', enable_events = True), sg.Button('Via QNS', key = '-QNS_INPUT-', enable_events = True)]
                   ]
sg_target_frame = sg.Frame("Target Position and Desired Angles", sg_target_layout, key = '-D_COORDS-', s = (500, 250))

left_col =  [[rotation_frame], [sg_coords_frame], [sg_target_frame], [sg.Button(button_text = "Calibrate", enable_events=True, key = '-CALIBRATE-'), sg.Button(button_text = "Aim", enable_events=True, key = '-AIM-'), sg.Exit()]]

# layout = [  
#             [rotation_frame, right_col],
#             [sg_coords_frame],
#             [sg_target_frame],
#             [],
#             [sg.Button(button_text = "Calibrate", enable_events=True, key = '-CALIBRATE-'), sg.Button(button_text = "Aim", enable_events=True, key = '-AIM-'), sg.Exit()]
#          ]
layout = [
            [sg.Column(left_col), sg.Column(right_col)]
        ]




window = sg.Window("GTNG", layout)

while True:
    #time.sleep(delta_time)
    event, values = window.read(timeout = 20)
    print(event, values)


        
    if event == "Exit" or event == sg.WIN_CLOSED or event in (None, 'Exit'):
        try:
            Servo.close()
            break
        except:
            break
        
    #Calibration Button Pressed with arduino connected
    if event == "-CALIBRATE-" and arduino_connected == True and qns_connected == True:
        #do the calibration
        calibration_success, yaw_centre, pitch_centre = calibration(arduino, rotation[2][0], rotation[0][0], rotation[1][0])
        if calibration_success == True:
            sg.popup("Calibration Success!", keep_on_top=True)
            ##CALIBRATION. DO THIS

        else:
            sg.popup("Calibration not successful")   
    elif event == "-CALIBRATE-" and arduino_connected == False and qns_connected == True:
        sg.popup("Please connect to Arduino", keep_on_top=True)
        calibration_success = False
    elif event == "-CALIBRATE-" and arduino_connected == True and qns_connected == False:
        sg.popup("Please connect to QNS", keep_on_top=True)
        calibration_success = False
    elif event == "-CALIBRATE-" and arduino_connected == False and qns_connected == False:
        sg.popup("Please connect to QNS and Arduino", keep_on_top=True)
        calibration_success = False
    
    #Connection to arduino
    if event == "-A_CONNECT-":
        try:
            arduino_port = values['-PORT_ADD-']
            arduino = serial.Serial(port=arduino_port, baudrate=115200, timeout=.1)
            arduino_connected = True
            sg.popup("Connection Successfull", keep_on_top=True)
            window['-PORT_ADD-'].update(disabled = True)
            window['-A_CONNECT-'].update(disabled = True)
            window['-A_CONNECTED_STATUS-'].update(value = 'True')
        except:
            sg.popup("Connection Unsuccessfull: Please re-enter the port exacty as you see it", keep_on_top=True)
            
    #Connection to QNS
    if event == "-QNS_CONNECT-" and qns_connected == False:
        try:
            Servo = MoCapStream.MoCap(stream_type="6d")
            sg.popup("QNS Connection Successfull: Tracking 6D object...", keep_on_top=True)
            qns_connected = True
            window['-QNS_CONNECTED_STATUS-'].update(value = 'True')
        except:
            sg.popup("QNS Connection Unsuccessfull: Please re-enter the IP exacty as you see it", keep_on_top=True)
            

    #targeting manual input to centre location
    if event == "-MANUAL_INPUT-":
        choice, target = make_target_popup()
        window['-T_X-'].update(value = target[x])
        window['-T_Y-'].update(value = target[y])
        window['-T_Z-'].update(value = target[z])
    
    #Targeting with QNS system to location
    if event == '-QNS_INPUT-':
        sg.popup("QNS Targeting Unsuccessfull: FUNCTIONALITY NOT ADDED", keep_on_top=True)
    
    #Aiming at the desired target location
    if event == '-AIM-' and calibration_success == True:
        if target[0] == np.nan:
            sg.popup("Please enter a valid target location", keep_on_top=True)
        else:
            yaw_angle, pitch_angle = aim_at(target, servo)
            window['-D_YAW_FROM_ZERO-'].update(value = yaw_angle)
            window['-D_PITCH_FROM_ZERO-'].update(value = pitch_angle)
    elif event == '-AIM-' and calibration_success == False:
        sg.popup("Please calibrate first", keep_on_top=True)
        
    
    #Remember to update the rotation matrix from QNS and output this here
    # for col in range(3):
    #     for row in range(3):
    #         key = '-R' + str(row) + str(col) + '-'
    #         window[key].update(value = rotation[row][col])
    #Remember to pull the GTNG's position too!
    #window['-O_X-'].update(value = servo[x])
    #window['-O_Y-'].update(value = servo[y])
    #window['-O_Z-'].update(value = servo[z])

    if qns_connected == True:
        index = 0
        #updates the rotation matrix
        for row in range(3):
            for col in range(3):
                rotation[row][col] = Servo.rotation[0][index]
                index = index + 1
                key = '-R' + str(row) + str(col) + '-'
                window[key].update(value=rotation[row][col])
        #updates the sensed yaw and pitch
        yaw_angle, pitch_angle = angles_from_rot(rotation[2][0], rotation[0][0], rotation[1][0])
        window['-YAW-'].update(value=np.degrees(yaw_angle))
        window['-PITCH-'].update(value=np.degrees(pitch_angle))
        # Remember to pull the GTNG's position too!
        window['-O_X-'].update(value=Servo.position[0])
        window['-O_Y-'].update(value=Servo.position[1])
        window['-O_Z-'].update(value=Servo.position[2])
        window.refresh()
            
            
window.close()


# In[ ]:


window.close()


# In[ ]:


# sg_rotation_layout.append(sg.Table(values = rotation, headings = ['1', '2', '3'],
#                                    auto_size_columns=True,
#                                    display_row_numbers=False,
#                                    justification='centre', 
#                                    key='-ROT_TABLE-',
#                                    selected_row_colors='red on white',
#                                    enable_events=True,
#                                    expand_x=True,
#                                    expand_y=True,
#                                    enable_click_events=False))

