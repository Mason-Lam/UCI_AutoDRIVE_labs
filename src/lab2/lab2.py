#!/usr/bin/env python

# Import libraries
from typing import Any
import numpy as np
import socketio
import eventlet
from flask import Flask
import autodrive
import math
import time

################################################################################

class PIDController:
    
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def reset(self):
        self.prev_error = 0
        self.integral = 0

    def update(self, setpoint, measurement, dt):
        error = setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        if (error - self.prev_error) > 0.2:
            derivative = 0
        
        # if derivative != 0: print("Derivative: ", derivative * self.kd)
        # derivative = max(min(derivative, 10), -10)  # Clamp derivative to [-10, 10]
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error
        return output
    
controller = PIDController(kp=0.5, ki=0.0, kd=0)

prevTime = 0

# Initialize vehicle(s)
f1tenth_1 = autodrive.F1TENTH()
f1tenth_1.id = "V1"
min_angle: float = -np.pi / 2.0  # radians
max_angle: float = np.pi / 2.0  # radians
disparity_size: float = 0.5  # meters TODO: set disparity size

# Initialize the server
sio = socketio.Server()

# Flask (web) app
app = Flask(__name__)  # '__main__'


# Registering "connect" event handler for the server
@sio.on("connect")
def connect(sid, environ):
    prevTime = time.monotonic()
    print("Connected!")


def index_to_angle(index: int, num_points: int) -> float:
    angle_increment = (max_angle - min_angle) / (num_points - 1)
    angle = min_angle + index * angle_increment
    return angle


def extend_disparity(lidar_range_array: np.ndarray[Any] | None, threshold: float):
    prevDist = lidar_range_array[0]
    if prevDist == float("inf"):
        prevDist = 10.0  # arbitrary large distance
    startIndex = 0
    for i in range(1, len(lidar_range_array)):
        currDist = lidar_range_array[i]
        if currDist == float("inf"):
            currDist = 10.0  # arbitrary large distance
            lidar_range_array[i] = currDist
        if currDist > prevDist and currDist - prevDist > threshold:
            lidar_range_array[i] = prevDist
        else:
            prevDist = currDist
            startIndex = i
        
        startAngle = index_to_angle(startIndex, lidar_range_array.size)
        currAngle = index_to_angle(i, lidar_range_array.size)
        if abs(currAngle - startAngle) * prevDist > disparity_size:
            prevDist = float('inf')
            startIndex = i

    prevDist = lidar_range_array[-1]
    startIndex = lidar_range_array.size - 1
    for i in range(len(lidar_range_array) - 1, -1, -1):
        currDist = lidar_range_array[i]
        if currDist > prevDist and currDist - prevDist > threshold:
            lidar_range_array[i] = prevDist
        else:
            prevDist = currDist
            startIndex = i
        
        startAngle = index_to_angle(startIndex, lidar_range_array.size)
        currAngle = index_to_angle(i, lidar_range_array.size)
        if abs(currAngle - startAngle) * prevDist > disparity_size:
            prevDist = float('inf')
            startIndex = i



def compute_speed(center_dist: float, target_dist: float, target_angle: float) -> float:
    degrees = math.degrees(target_angle)

    if target_dist > 5 and abs(degrees) < 5:
        return 0.9, 0.2
    return 0.6, 0.5


# Registering "Bridge" event handler for the server
@sio.on("Bridge")
def bridge(sid, data):
    if data:
        global prevTime
        f1tenth_1.parse_data(data)

        lidar_range_array = f1tenth_1.lidar_range_array[
            f1tenth_1.lidar_range_array.size // 6 : -f1tenth_1.lidar_range_array.size
            // 6
        ]

        # print(lidar_range_array)

        extend_disparity(lidar_range_array, threshold=0.05)  # TODO: set threshold

        # print(lidar_range_array)

        best_point_index = np.argmax(lidar_range_array)  # TODO

        best_point_angle = index_to_angle(best_point_index, lidar_range_array.size)

        # print(math.degrees(best_point_angle))

        prevTime = time.monotonic()

        target_dist = lidar_range_array[best_point_index]
        
        center_dist = lidar_range_array[lidar_range_array.size // 2]

        speed, proportional_gain = compute_speed(center_dist, target_dist, best_point_angle)

        f1tenth_1.steering_command = best_point_angle * proportional_gain

        # print("Distance", target_dist)

        f1tenth_1.throttle_command = speed

        ########################################################################

        json_msg = f1tenth_1.generate_commands()  # Generate vehicle 1 message

        try:
            sio.emit("Bridge", data=json_msg)
        except Exception as exception_instance:
            print(exception_instance)


################################################################################

if __name__ == "__main__":
    app = socketio.Middleware(
        sio, app
    )  # Wrap flask application with socketio's middleware
    eventlet.wsgi.server(
        eventlet.listen(("", 4567)), app
    )  # Deploy as an eventlet WSGI server
