from pymavlink import mavutil
from dronekit import *
import cv2
import numpy as np
from simple_pid import PID
program_start = time.perf_counter()
lower, upper = np.array([23, 44, 0]), np.array([52, 103, 245])

#############################################################################################################################
def process_frame(frame, lower, upper):
    start = time.perf_counter()
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
    #if area > 200:
    if area > 200:
        detection = True
        box_x, box_y, box_w, box_h = cv2.boundingRect(largest_contour)
        cx, cy = box_x + box_w // 2, box_y + box_h // 2
        cv2.rectangle(original, (box_x, box_y), (box_x + box_w, box_y + box_h), (255, 0, 0), 2)
        cv2.line(original, (sx, sy), (cx, cy), (255, 0, 0), 5)
        cv2.circle(original, (cx, cy), 1, (255, 0, 0), 10)
        cv2.circle(original, (sx, sy), 1, (0, 255, 0), 10)
        cv2.rectangle(original, (box_x, box_y), (box_x + box_w, box_y + box_h), (255, 0, 0), 2)
        focal_l = 80.75 * 0.8 / 0.067
        rho = 65.5 * focal_l / (box_w*1000)
        phi, beta = np.arctan((sy - cy) / rho) * 180 / np.pi, np.arctan((sx - cx) / rho) * 180 / np.pi
        print(rho)
        
        #current_time = time.perf_counter()
        # timestamp, processing time, FPS
        #print(str(current_time - program_start) + ", " + str(current_time - start) + ", " + str(1/(current_time - start)))
        # print('Time: ', time.perf_counter() - start, 'FPS: ', 1 / (time.perf_counter() - start))
    else:
        detection = False
        rho, phi, beta = 0, 0, 0
        
    return detection, original, rho, phi, beta


#############################################################################################################################
cap = cv2.VideoCapture(0)
frame_count = 0
while frame_count < 100:
    ret, img = cap.read()
    if ret:
        frame_count += 1
        detection, original, rho, phi, beta = process_frame(img, lower, upper)
        

        cv2.imshow('Video Stream', original)
        # if detection and rho>follow_dist:
        #     move_drone(vx, beta, 0)
        #     yaw_track(beta, yaw_rate)
        # elif detection and rho<=follow_dist:
        #     move_drone(0, beta, 0)
        #     yaw_track(beta, yaw_rate)
    # alt_error = vehicle.location.global_relative_frame.alt - target_alt
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()


