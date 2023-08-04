import MoCapStream
import serial
import time
from numpy import array
from numpy.linalg import norm
import numpy as np

x = 0
y = 1
z = 2

gtng_back = 3
gtng_front = 1

# ***Possible Tracking Types:****
# Following: The servos continually update the target position based on pulling data from the qualisys
# Fixed_Area: The servos update the target position when it is inside a space
# Fixed_User_Input: The target is provided by the user
# Fixed_Input: The initial target is provided by the qualisys with No Tracking

Tracking_Type = "Following"

#Servo = MoCapStream.MoCap(stream_type='3d_labelled')
Servo = MoCapStream.MoCap(stream_type="6d")
while True:
    print(Servo.position)
    time.sleep(0.01)

arduino = serial.Serial(port='COM4', baudrate=115200, timeout=.1)
time.sleep(1)

# *******************CALIBRATION***********************************
# Start the Calibration Process
Calibration_success = False
arduino.write(bytes('c', 'utf-8'))  # Sends calibration command
time.sleep(0.5)  # waits for the servo to move
#arduino.write(bytes('', 'utf-8'))  # Sends calibration command

# Captures servo yaw centre angle and convert to degrees
while Calibration_success == False:
    #try:
        yaw_centre = (np.arctan2((Servo.marker[gtng_front][y] - Servo.marker[gtng_back][y]),
                                 (Servo.marker[gtng_front][x] - Servo.marker[gtng_back][x])) * 4068) / 71

        # waits for servos to move
        time.sleep(0.5)
        # Captures servo pitch centre angle and convert to degrees
        pitch_centre = (np.arctan2( np.sqrt( np.square(Servo.marker[gtng_front][x] - Servo.marker[gtng_back][x]) + np.square(Servo.marker[gtng_front][y] - Servo.marker[gtng_back][y]) ),
            (Servo.marker[gtng_front][z] - Servo.marker[gtng_back][z])) * 4068) / 71

        yaw_max = yaw_centre + 60
        yaw_min = yaw_centre - 60

        pitch_max = pitch_centre + 60
        pitch_min = pitch_centre - 60

        Calibration_success = True
    # except:
    #     print('Cannot calibrate: Issue occured')
    #     time.sleep(1)
#
# # ******************END OF CALIBRATION****************************
print("CALIBRATION COMPLETE:")
print("YAW: " + str(yaw_min) + ", " + str(yaw_centre) + ", " + str(yaw_max))
print("PITCH: " + str(pitch_min) + ", " + str(pitch_centre) + ", " + str(pitch_max))

# ******************Start Normal Run Time*************************
# delta_time = 0.05;
# projectile_velocity = 500;
# if Tracking_Type == "Following":
#     # set up initial values. Will take four iterations to get everything moving so arrays arent set to zero. I am doing this manually as i dont want zeros to make their way into the control for the servos and solenoid and introduce instability
#     try:
#         target = objects[4]
#         last_target = [target.marker[0][x], target.marker[0][y], target.marker[0][z]]
#         T_dot = [0, 0, 0]
#         last_T_dot = [0, 0, 0]
#         T_accel = [0, 0, 0]
#         last_T_accel = [0, 0, 0]
#         time.sleep(delta_time)
#
#         current_target = [target.marker[0][x], target.marker[0][y], target.marker[0][z]]
#         T_dot = [(current_target[x] - last_target[x]) / delta_time, (current_target[y] - last_target[y]) / delta_time,
#                  (current_target[z] - last_target[z]) / delta_time]
#         last_T_dot = [0, 0, 0]
#         T_accel = [0, 0, 0]
#         last_T_accel = [0, 0, 0]
#         time.sleep(delta_time)
#
#         last_target = current_target
#         current_target = [target.marker[0][x], target.marker[0][y], target.marker[0][z]]
#         last_T_dot = T_dot
#         T_dot = [(current_target[x] - last_target[x]) / delta_time, (current_target[y] - last_target[y]) / delta_time,
#                  (current_target[z] - last_target[z]) / delta_time]
#         T_accel = [(T - dot[x] - last_T_dot[x]) / delta_time, (T - dot[y] - last_T_dot[y]) / delta_time,
#                    (T - dot[z] - last_T_dot[z]) / delta_time]
#         last_T_accel = [0, 0, 0]
#         time.sleep(delta_time)
#
#         last_target = current_target
#         current_target = [target.marker[0][x], target.marker[0][y], target.marker[0][z]]
#         last_T_dot = T_dot
#         T_dot = [(current_target[x] - last_target[x]) / delta_time, (current_target[y] - last_target[y]) / delta_time,
#                  (current_target[z] - last_target[z]) / delta_time]
#         last_T_accel = T_accel
#         T_accel = [(T - dot[x] - last_T_dot[x]) / delta_time, (T - dot[y] - last_T_dot[y]) / delta_time,
#                    (T - dot[z] - last_T_dot[z]) / delta_time]
#         time.sleep(delta_time)
#     except:
#         print("Tracking Type 'Following': Failed in first four iterations. Markers cannot be seen")
#
#     while True:
#         try:
#             # Update all kinematic values
#             # update the targets
#             last_target = current_target
#             current_target = [target.marker[0][x], target.marker[0][y], target.marker[0][z]]
#             # update and calculate the velocity
#             last_T_dot = T_dot
#             T_dot = [(current_target[x] - last_target[x]) / delta_time,
#                      (current_target[y] - last_target[y]) / delta_time,
#                      (current_target[z] - last_target[z]) / delta_time]
#             # update and calculate acceleration
#             last_T_accel = T_accel
#             T_accel = [(T - dot[x] - last_T_dot[x]) / delta_time, (T - dot[y] - last_T_dot[y]) / delta_time,
#                        (T - dot[z] - last_T_dot[z]) / delta_time]
#             # interception calculation with the duty cycles being returned. Order of Pitch then Yaw
#             dutyCycles = interception([Servo.marker[0][x], Servo.marker[0][y], Servo.marker[0][z]], current_target,
#                                       T_dot, T_accel, projectile_velocity, yaw_min, pitch_min)
#
#             # Pausing the operation to allow servos to move and a new position to be gathered
#             arduino.write(bytes(dutyCycles, 'utf-8'))
#             time.sleep(delta_time)
#
#
#
#         except:
#             print("Tracking Type 'Following': Failed in general run time. Marker may not be seen")


# A function to calculate the interception point between the tracked target and the
def interception(servo, target, velocity, acceleration, projectile_velocity, yaw_min, pitch_min):
    # Step 1: Calculate Displacement between target and servo
    displacement = [target[x] - servo[x], target[y] - servo[y], target[z] - servo[z]]
    # Step 2: Estimate time to reach T by projectile
    t1 = np.sqrt(displacement[x] ^ 2 + displacement[y] ^ 2 + displacement[z] ^ 2) / projectile_velocity
    # Step 3: Estimate future position of T at time t1
    for i in range(3):
        T_future[i] = target[i] + velocity[i] * t1 + (acceleration[i] * t1) / 2
    # Step 4: Calculate new displacment from servo to future posn
    future_displacement = [T_future[x] - servo[x], T_future[y] - servo[y], T_future[z] - servo[z]]
    # Step 5: Estimate the additional time required for the projectile to reach the future position
    t2 = np.sqrt(
        future_displacement[x] ^ 2 + future_displacement[y] ^ 2 + future_displacement[z] ^ 2) / projectile_velocity
    # Step 6: lead time is total time to reach target. T1 + t2
    lead_time = t1 + t2
    # step 7: aim at the position now given
    for i in range(3):
        T_aim[i] = target[i] + velocity[i] * lead_time + (acceleration[i] * lead_time) / 2

    # calculate the angles between the points
    xy_angle = (np.arctan2(T_aim[y] - servo[y], T_aim[x] - servo[x]) * 4068) / 71
    zx_angle = (np.arctan2(np.sqrt((T_aim[x] - servo[x]) ^ 2 + (T_aim[y] - servo[y]) ^ 2),
                           (T_aim[z] - servo[z])) * 4068) / 71

    yaw_angle = np.absolute(yaw_min) + xy_angle
    pitch_angle = np.absolute(pitch_min) + (90 - zx_angle)

    pitch_duty_cycle = convert_and_bound(pitch_angle)
    yaw_duty_cycle = convert_and_bound(yaw_angle)

    return [pitch_duty_cycle, yaw_duty_cycle]


# function to take two positions and return the angles between them
def find_angle(posn1, posn2):
    distance = np.sqrt((posn2[x] - posn1[x]) ^ 2 + (posn2[y] - posn1[y]) ^ 2 + (posn2[z] - posn1[z]) ^ 2)
    xy_angle = (np.arctan2(posn2[y] - posn1[y], posn2[x] - posn1[x]) * 4068) / 71
    zx_angle = (np.arctan2(np.sqrt((posn2[x] - posn1[x]) ^ 2 + (posn2[y] - posn1[y]) ^ 2),
                           (posn2[z] - posn1[z])) * 4068) / 71
    return [xy_angle, zx_angle]


def convert_and_bound(angle):
    if angle <= 120 and angle >= 0:
        dutyPeriod = convert_to_duty_cycle(angle)
    elif angle < 0:
        angle = 0
        dutyPeriod = convert_to_duty_cycle(angle)
    elif angle > 120:
        angle = 120
        dutyPeriod = convert_to_duty_cycle(angle)
    return dutyPeriod


def convert_to_duty_cycle(angle):
    degreesInMs = 8.3333
    dutyCycle = 1000 + angle * degreesInMs
    return dutyCycle

