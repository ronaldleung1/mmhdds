from pymavlink import mavutil
from dronekit import *
import cv2
import numpy as np
from simple_pid import PID


################################################################################################
follow_dist = .3
vx, yaw_rate = .6, 0.2
vz = 0.4
target_alt = 1
lower, upper = np.array([29, 86, 6]), np.array([64, 255, 255])
delta_target = 0.2 # .2 meters below the tennis ball
delta_tolerance = 0.1
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
    direction = -1 #what is direction? - figure out
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


def move_drone(vx, vz, yaw, yaw_rate): # don't rlly understand - figure out how it works
    # Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
    # location in the North, East, Down frame.
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
        0b00111000111,  # type_mask
        0, 0, 0,  # x, y, z position in m
        vx, 0, vz,  # x, y, z velocity in m/s  (not used)
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        yaw, yaw_rate)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)


def process_frame(frame, lower, upper):
    original = frame.copy()
    sx, sy = len(frame[0, :]) // 2, len(frame[:, 2]) // 2
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    frame = cv2.inRange(frame, lower, upper)
    contours, _ = cv2.findContours(frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if len(contours) > 0:
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
    else:
        area = 0
    if area > 200:
        detection = True # there is detection of the tennis ball or object
        box_x, box_y, box_w, box_h = cv2.boundingRect(largest_contour)
        cx, cy = box_x + box_w // 2, box_y + box_h // 2
        cv2.rectangle(original, (box_x, box_y), (box_x + box_w, box_y + box_h), (255, 0, 0), 2)
        cv2.line(original, (sx, sy), (cx, cy), (255, 0, 0), 5)
        cv2.circle(original, (cx, cy), 1, (255, 0, 0), 10)
        cv2.circle(original, (sx, sy), 1, (0, 255, 0), 10)
        cv2.rectangle(original, (box_x, box_y), (box_x + box_w, box_y + box_h), (255, 0, 0), 2)
        focal_l = 35 * 1000 / 65.5
        rho = 65.5 * focal_l / (box_w*1000)
        phi, beta = np.arctan((sy - cy) / rho) * 180 / np.pi, np.arctan((sx - cx) / rho) * 180 / np.pi
        delta = sy - cy # vertical distance between the center of the camera and the tennis ball
    else:
        detection = False # there isn't detection of the tennis ball or object
        rho, phi, beta = 0, 0, 0
    return detection, original, rho, phi, beta, delta


#############################################################################################################################
vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)
arm_drone()
time.sleep(2)
drone_takeoff(target_alt)
time.sleep(1)

# For testing
# for x in range(1):
#     move_drone(.5,0,0)
#     time.sleep(1)




cap = cv2.VideoCapture(0)
while True:
    ret, img = cap.read()
    
    if ret:
        detection, original, rho, phi, beta, delta = process_frame(img, lower, upper)
        cv2.imshow('Video Stream', original)
        if detection:
            if (rho > follow_dist): # drone is too far away
                vx_output = vx # move drone forward
            else:
                vx_output = 0 # stop drone
            
            if (delta - delta_target) > delta_tolerance: # drone altitude is above target
                vz_output = vz # positive output is down
            elif (delta_target - delta) > delta_tolerance: # drone altitude is below target
                vz_output = -1 * vz
            else: # drone altitude is within tolerance
                vz_output = 0
            
            move_drone(
                vx_output,
                vz_output,
                beta,
                0)
            yaw_track(beta, yaw_rate) # yaw rate is set to zero here - might be a problem when trying to track the ball
    # alt_error = vehicle.location.global_relative_frame.alt - target_alt
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
