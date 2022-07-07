from pymavlink import mavutil
from dronekit import *
import cv2
import numpy as np
from simple_pid import PID


################################################################################################
follow_dist = .3
vx, yaw_rate = .6, 0
target_alt = 1
lower, upper = np.array([29, 86, 6]), np.array([64, 255, 255])
################################################################################################


def connect_drone(port, wait_ready=True, baud_rate=57600):
    vehicle = connect(port, wait_ready, baud_rate)
    print('Connection succesful')
    return vehicle


def arm_drone():
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    print("Arming motors")
    vehicle.armed = True


def drone_takeoff(altitude_target):
    if not vehicle.armed:
        print('Drone is not armed, aborting')
        quit()
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.simple_takeoff(altitude_target)  # Take off to target altitude
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= altitude_target*0.90:
            print("Reached target altitude")
            break
        time.sleep(1)


def yaw_track(yaw_angle, rate=yaw_rate):
    direction = -1
    if yaw_angle < 0:
        yaw_angle = yaw_angle*-1
        direction = 1

    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        yaw_angle,    # param 1, yaw in degrees
        rate,          # param 2, yaw speed deg/s
        direction,          # param 3, direction -1 ccw, 1 cw
        1,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


def move_drone(vx, yaw, yaw_rate):
    # Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
    # location in the North, East, Down frame.
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
        0b00111000111,  # type_mask
        0, 0, 0,  # x, y, z position in m
        vx, 0, 0,  # x, y, z velocity in m/s  (not used)
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        yaw, yaw_rate)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)


#############################################################################################################################
vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)
arm_drone()
time.sleep(2)
drone_takeoff(target_alt)
time.sleep(1)

