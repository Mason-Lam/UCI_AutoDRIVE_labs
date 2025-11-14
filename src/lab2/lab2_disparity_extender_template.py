#!/usr/bin/env python

# Import libraries
from typing import Any
import numpy as np
import socketio
import eventlet
from flask import Flask
import autodrive

################################################################################

# Initialize vehicle(s)
f1tenth_1 = autodrive.F1TENTH()
f1tenth_1.id = "V1"
min_angle: float = -np.pi / 2.0  # radians
max_angle: float = np.pi / 2.0  # radians
disparity_size: float = 0.0  # meters TODO: set disparity size

# Initialize the server
sio = socketio.Server()

# Flask (web) app
app = Flask(__name__)  # '__main__'


# Registering "connect" event handler for the server
@sio.on("connect")
def connect(sid, environ):
    print("Connected!")


def index_to_angle(index: int, num_points: int) -> float:
    angle_increment = (max_angle - min_angle) / (num_points - 1)
    angle = min_angle + index * angle_increment
    return angle


def extend_disparity(lidar_range_array: np.ndarray[Any] | None, threshold: float):
    pass  # TODO


def compute_speed(target_distance: float) -> float:
    return 1.0  # You can ramp based on target_distance, TODO


# Registering "Bridge" event handler for the server
@sio.on("Bridge")
def bridge(sid, data):
    if data:
        f1tenth_1.parse_data(data)

        lidar_range_array = f1tenth_1.lidar_range_array[
            f1tenth_1.lidar_range_array.size // 6 : -f1tenth_1.lidar_range_array.size
            // 6
        ]

        extend_disparity(lidar_range_array, threshold=0.0)  # TODO: set threshold

        best_point_index = 0  # TODO

        best_point_angle = index_to_angle(best_point_index, lidar_range_array.size)
        f1tenth_1.steering_command = best_point_angle / (3.0 * np.pi / 4.0)

        target_distance = lidar_range_array[best_point_index]
        f1tenth_1.throttle_command = compute_speed(target_distance)

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
