import pickle

calibration_data = {
    "header": {
        "stamp": {
            "sec": 1760574285,
            "nanosec": 862491129
        },
        "frame_id": "camera_optical"
    },
    "height": 480,
    "width": 856,
    "distortion_model": "plumb_bob",
    "d": [0.004974, -0.00013, -0.001212, 0.002192, 0.0],
    "k": [
        537.292878, 0.0, 427.331854,
        0.0, 527.000348, 240.226888,
        0.0, 0.0, 1.0
    ],
    "r": [
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    ],
    "p": [
        539.403503, 0.0, 429.275072, 0.0,
        0.0, 529.838562, 238.941372, 0.0,
        0.0, 0.0, 1.0, 0.0
    ],
    "binning_x": 0,
    "binning_y": 0,
    "roi": {
        "x_offset": 0,
        "y_offset": 0,
        "height": 0,
        "width": 0,
        "do_rectify": False
    }
}

with open("ros_data.pkl", "wb") as f:
    pickle.dump(calibration_data, f)

print("Archivo 'calibration_data.pkl' generado correctamente.")
