#!/usr/bin/env python

# Import libraries
import socketio
import eventlet
from flask import Flask
import autodrive
import math

import numpy as np

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

################################################################################


prevOutput = 1

# Initialize vehicle(s)
f1tenth_1 = autodrive.F1TENTH()
f1tenth_1.id = "V1"

# Initialize the server
sio = socketio.Server()

# Flask (web) app
app = Flask(__name__)  # '__main__'

controller = PIDController(kp=0.9, ki=0.0, kd=0.1)

offset = 5

offset2 = 180

offset3 = 900

# Registering "connect" event handler for the server
@sio.on("connect")
def connect(sid, environ):
    print("Connected!")


def degreesToIndex(range_array, degrees):
    angle_increment = 270 / len(range_array)
    index = int(degrees / angle_increment)
    return index

# Registering "Bridge" event handler for the server
@sio.on("Bridge")
def bridge(sid, data):
    if data:
        global prevOutput
        # Vehicle data
        f1tenth_1.parse_data(data)

        a = f1tenth_1.lidar_range_array[degreesToIndex(f1tenth_1.lidar_range_array, offset) + offset2]
        b = f1tenth_1.lidar_range_array[degreesToIndex(f1tenth_1.lidar_range_array, 0) + offset2]

        a2 = f1tenth_1.lidar_range_array[-int(math.copysign(1, prevOutput)) * degreesToIndex(f1tenth_1.lidar_range_array, offset) + offset3]
        b2 = f1tenth_1.lidar_range_array[degreesToIndex(f1tenth_1.lidar_range_array, 0) + offset3]

        # print(a, a2)

        # print(np.array(f1tenth_1.lidar_range_array).argmin())
        # print(b, b2)

        a = max(min(a, 10), -10)
        b = max(min(b, 10), -10)
        a2 = max(min(a2, 10), -10)
        b2 = max(min(b2, 10), -10)

        # if (a == float("Inf") or b == float("Inf") or a2 == float("Inf") or b2 == float("Inf")):
        #     print("INFINITY GENERATED")
        #     aIndex = degreesToIndex(f1tenth_1.lidar_range_array, offset) + offset2
        #     print("A:", a)
        #     print("NEW:", np.array(f1tenth_1.lidar_range_array)[aIndex - 20 : aIndex + 20])
        #     print("B:", b)
        #     print("A2:", a2)
        #     print("B2:", b2)
        #     json_msg = f1tenth_1.generate_commands()
        #     try:
        #         sio.emit("Bridge", data=json_msg)
        #     except Exception as exception_instance:
        #         print(exception_instance)
        #     return
        
        theta = math.radians(offset)

        # print(a * math.cos(theta) - b)
        # print(a2 * math.cos(theta) - b2)

        # print (a * math.sin(theta), a2 * math.sin(theta))

        alpha = math.atan2(a * math.cos(theta) - b, a * math.sin(theta))
        alpha2 = math.atan2(a2 * math.cos(theta) - b2, a2 * math.sin(theta))

        # print (math.degrees(alpha), math.degrees(alpha2))

        dRight = b * math.cos(alpha)
        dLeft = b2 * math.cos(alpha2)

        # print(dLeft, dRight)

        d = dRight - dLeft

        # print(d)

        output = controller.update(0, d, 0.05)

        prevOutput = output

        driveSpeed = 0.3

        if (abs(d) < 0.1):
            driveSpeed = 0.4

        if (abs(output) > 0.25):
            driveSpeed = 0.13

        # Vehicle control
        f1tenth_1.throttle_command = driveSpeed # [-1, 1]
        f1tenth_1.steering_command = output if abs(d) > 0.10 else 0  # generate your steering command

        ########################################################################

        json_msg = f1tenth_1.generate_commands()

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
