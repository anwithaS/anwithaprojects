import clr
import time
import csv
import System
from System import Array
from System.Collections.Generic import List
from collections import defaultdict
import numpy as np
import tkinter as tk
import pandas as pd


# references
clr.AddReference(r"C:\Users\RoSenCoLab-4\Documents\HBM\HBM Common API\API\Hbm.Api.Common.dll")
clr.AddReference(r"C:\Users\RoSenCoLab-4\Documents\HBM\HBM Common API\API\Hbm.Api.SensorDB.dll")
clr.AddReference(r"C:\Users\RoSenCoLab-4\Documents\HBM\HBM Common API\API\Hbm.Api.Scan.dll")
clr.AddReference(r"C:\Users\RoSenCoLab-4\Documents\HBM\HBM Common API\API\Hbm.Api.QuantumX.dll")


from Hbm.Api.Common import DaqEnvironment, DaqMeasurement
import Hbm.Api.Common.Entities as Entities
import Hbm.Api.Common as Common




#global variables
qx = None #qx: QuantumX device reference
env = None  #env: QuantumX environment reference
abd_angle = 0.0 #abd_angle: arm abduction angle in degrees.
elb_angle = 0.0 #elb_angle: elbow flexion angle in degrees (from full flexion)
center_x = center_y = window_width = window_height = 0 #center_x, center_y: center of the window
elbow_torque = None
shoulder_torque = None


#change angle to radians
def deg_to_rad(deg):
    return (deg/180)*np.pi #Convert to radians
#convert units from lbf to N and lbf-in to Nm
def convert_units(matrix):
    # Conversion factors
    Lb_N = 4.4452016       # lbf to N
    Lbin_Nm = 0.11298483   # lbf-in to Nm
    # Convert forces (first 3 values)
    matrix[0:3] = matrix[0:3] * Lb_N
    # Convert torques (last 3 values)
    matrix[3:6] = matrix[3:6] * Lbin_Nm
    return matrix
# X rotation matrix
def rot_x(theta):
    return np.array([
        [1, 0, 0],
        [0, np.cos(theta), -np.sin(theta)],
        [0, np.sin(theta), np.cos(theta)]
    ])
# Y rotation matrix
def rot_y(theta):
    return np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])
# Z rotation matrix
def rot_z(theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])
# translation matrix, the input is a 3x1 vector
def Px(r):
    return np.array([
        [0, -r[2], r[1]],
        [r[2], 0, -r[0]],
        [-r[1], r[0], 0]
    ])
# initialize the environment and connect to the device
def initialize_env():
    global qx, env
    env = DaqEnvironment.GetInstance() # Initialize DaqEnvironment (initializes scanning plugins)
    time.sleep(5)    # Wait a few seconds for devices to announce themselves
    devices = env.Scan() # Perform device scan
   
    qx = devices[0]  # reference and name the device
   
    sucess = env.Connect(qx)#connect the device
    #check if its connected or it has any problems
   
    if sucess:
        print(f"Sucessfully Connected")
    else:
        print("Failed to connect")
        exit()
#measures reading
def measure_snapshot():
    qx.ReadSingleMeasurementValueOfAllSignals() #reads current measurement snapshot
    signals = qx.GetAllSignals() #gets all the signals
    sig_val = {} #dictionary
    timestamp = None
    for sig in signals:
        name = sig.Name #gets the name
        time = sig.GetSingleMeasurementValue().Timestamp # gets the timestamp
        value = sig.GetSingleMeasurementValue().Value
        state = sig.GetSingleMeasurementValue().State
        if name not in sig_val: # only adds the first value that is read. assuming the first value is raw data and second is processed
            sig_val[name] = value #add to dictionary
        if timestamp is None: # only one value added for time.
            timestamp = time
    # remove the last 3 times since it is not needed for the script
    for _ in range(3):
        sig_val.popitem() #pop the last 3 values from the dictionary


    return(timestamp, sig_val) #return the timestamp and the dictionary with the values
#should return shoulder moment and elbow force
# the input is a 1x6 matrix of the raw data
def jacobian(r_data):
    jac_mat = np.eye(6) # identity matrix. Later will be replaced with the calibration matrix
    jacobian_data = r_data.reshape((6, 1)) #6x1 matrix of the raw data
    abd_angle = 0.0 #abd_angle: arm abduction angle in degrees.
    elb_angle = 0.0 #elb_angle: elbow flexion angle in degrees (from full flexion)
    arm_length = 1.0 #arm_length: length of the arm in meters
    z_offset = 0
    abd_angle = deg_to_rad(abd_angle) #Convert to radians
    elb_angle = deg_to_rad(elb_angle) #Convert to radians

    Fmjr = jac_mat @ jacobian_data # mutiply the calibration matrix and the raw data
    Fmjr = convert_units(Fmjr) # convert to N and Nm

    #convert to right hand coordinate system????
    Fmjr[[2, 5]] *= -1

    #elbow transformation matrix from the sensor translated to the elbow
   
    R = rot_z((np.pi / 2)) #rotation matrix around the z axis
   
    # jacobian matrix withe the rotation matrix and the translation matrix
    jacobian_to_elbow = np.block([
        [R, Px([0, 0, -z_offset]) @ R],
        [np.zeros((3, 3)), R]
    ])

    Fmjr = np.dot(jacobian_to_elbow.T, Fmjr) #multiply the transposed jacobian matrix and the force matrix

    #elbow transformation due to the arm abduction angle
    R = rot_y(abd_angle) #rotation matrix around the y axis
    #second jacobian matrix for the elbow rotation
    jacobian_2 = np.block([
        [R, Px([0, 0, 0]) @ R],
        [np.zeros((3, 3)), R]
    ])

    Elbow_FM = jacobian_2.T @ Fmjr #multiply the transposed jacobian matrix and the force matrix

    elbow_torque = float(Elbow_FM[3].item()) #get the elbow torque from the matrix in the upward direction, which in this case is the Mx
   
    #shoulder transformation matrix
    R = rot_x(elb_angle - np.pi / 2) #rotation matrix around the x axis
    y = arm_length * np.sin(elb_angle - np.pi / 2)
    z = arm_length * np.cos(elb_angle - np.pi / 2)


    #jacobian matrix with the rotation matrix and the translation matrix for shoulder
    jacobian_to_shoulder = np.block([
        [R, Px([0, y, z]) @ R],
        [np.zeros((3, 3)), R]
    ])


    Shoulder_FM = jacobian_to_shoulder.T @ Elbow_FM #multiply the transposed jacobian matrix and the elbow force matrix


    shoulder_torque = float(Shoulder_FM[4].item()) #get the shoulder moment from the matrix around the upward direction, which in this case is the My

    return elbow_torque,shoulder_torque


def draw_all(event = None): #input is the elbow force and shoulder moment
    global elbow_torque, shoulder_torque

    canvas.delete("all") # delete all the previous drawings
    global center_x, center_y, window_width, window_height # declare global variables and indicate updating
    window_width = canvas.winfo_width() # get the current width of the window
    window_height = canvas.winfo_height() # get the current height of the window
    center_x = window_width // 2  # calculate the center x coordinate of the window
    center_y = window_height // 2 # calculate the center y coordinate of the window

    # Draw plus
    line_size = 10
    canvas.create_line(center_x, center_y - line_size, center_x, center_y + line_size, width=1)
    canvas.create_line(center_x - line_size, center_y, center_x + line_size, center_y, width=1)

    # Draw background circle
    radius = 100
    canvas.create_oval(center_x - radius, center_y - radius, center_x + radius, center_y + radius, outline="black", width=1)

    # Draw center line
    canvas.create_line(0, center_y, window_width, center_y, width=2)


    # Draw updated forces
        # Shoulder circle
    canvas.create_oval(center_x - shoulder_torque, center_y - shoulder_torque,
                           center_x + shoulder_torque, center_y + shoulder_torque,
                           outline="red", width=1)
        # this circle visually represents the magnitude of the shoulder moment or force, scaled as a visual aid.
        # Elbow line
    offset = elbow_torque
    y = center_y - offset
    canvas.create_line(0, y, window_width, y, fill="red", width=1)


    window.update_idletasks() #finish pending drawings
    window.update() # process window events


#save the data to a csv file
#input is a list of lists with the data to be saved
def save_full_data(full_data, filename="final_measurements.csv"):
    headers = ["Timestamp", "Fx", "Fy", "Fz", "Mx", "My", "Mz", "ElbowTorque", "ShoulderTorque"]


    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(headers) # write the header
        writer.writerows(full_data) # write the data
    print("final measurements added")

# initialize GUI
# Create window
window = tk.Tk() # Create a window
window.title("Quantum DAQ") # Set window title
dim_x, dim_y = 600, 600 # Set window dimensions
window.geometry(f"{dim_x}x{dim_y}") # Set window size
# Create canvas
canvas = tk.Canvas(window, width=dim_x, height=dim_y) # Create a canvas
canvas.pack(fill=tk.BOTH, expand=True) # Fill the window with the canvas, expand on both sides
# Bind the canvas to resize event
canvas.bind("<Configure>", draw_all)

def main():
    initialize_env() #initialize the environment and connect to the device

    full_data = [] #initialize the data list
    start_time = time.time()
    duration = 60 # seconds
    while time.time() - start_time < duration:
        global elbow_torque, shoulder_torque

        timestamp, raw_data = measure_snapshot() #measure
        raw_data_matrix = np.array(list(raw_data.values())) #make into a matrix
        elbow_torque, shoulder_torque = jacobian(raw_data_matrix) # calculate the jacobian matrix and get the elbow force and shoulder moment

        draw_all() # visualize the data
       
        row = [timestamp] + list(raw_data.values()) + [elbow_torque,shoulder_torque]
        full_data.append(row)

        time.sleep(1) # wait for 5 seconds before the next measurement


    save_full_data(full_data)
    window.mainloop()
if __name__ == "__main__":
    main()












