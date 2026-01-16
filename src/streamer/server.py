from typing import Dict, TypedDict, Tuple, List
import time
import json
from threading import Thread
from pathlib import Path
from copy import deepcopy

from flask import Flask, stream_with_context, Response
import cv2


class Odometry(TypedDict):
    x: float
    y: float
    theta: float
    circles: List[Dict]
    lines: List[Dict]

class Streamer:
    def __init__(self) -> None:
        self.__json_data: Dict = {}
        self.__img: cv2.Mat = None
        self.__odometry: Odometry = {
            "x": 0,
            "y": 0,
            "theta": 0,
            "circles": [],
            "lines": [],
        }

        self.__app = Flask(__name__)
        self.__app.add_url_rule("/stream_img", "stream_img", self.__stream_img)
        self.__app.add_url_rule("/stream_data", "stream_data", self.__stream_data)
        self.__app.add_url_rule(
            "/stream_odometry", "stream_odometry", self.__stream_odometry
        )
        self.__app.add_url_rule("/", "index", self.__index)

        self.__thread: Thread = None
        self.run_app()

    """ SETTERS """

    def set_data(self, data: Dict) -> None:
        data_copy = deepcopy(data)
        self.__json_data = data_copy

    def set_img(self, img: cv2.Mat) -> None:
        img_copy = img.copy()
        self.__img = img_copy

    def set_odometry(self, odometry: Odometry) -> None:
        odometry_copy = deepcopy(odometry)
        self.__odometry = odometry_copy

    def update_odom_state(self, x: float, y: float, theta: float) -> None:
        self.__odometry["x"] = x
        self.__odometry["y"] = y
        self.__odometry["theta"] = theta

    def update_circles(self, circles: List[Tuple[float, float, str]]) -> None:
        self.__odometry["circles"] = [
            {"x": circle[0], "y": circle[1], "c": circle[2]} for circle in circles
        ]

    def update_lines(self, lines: List[Tuple[float, float, float, float, str]]) -> None:
        self.__odometry["lines"] = [
            {"x1": line[0], "y1": line[1], "x2": line[2], "y2": line[3], "c": line[4]}
            for line in lines
        ]

    """ STREAMING ENDPOINTS """

    def __stream_data(self) -> Response:
        def generate():
            while True:
                json_str = json.dumps(self.__json_data)
                yield f"data: {json_str}\n\n"
                time.sleep(0.1)

        return Response(stream_with_context(generate()), mimetype="text/event-stream")

    def __stream_img(self) -> Response:
        def generate():
            while True:
                if self.__img is not None:
                    _, img_encoded = cv2.imencode(".jpg", self.__img)
                    img_bytes = img_encoded.tobytes()
                    yield (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n\r\n" + img_bytes + b"\r\n"
                    )
                time.sleep(0.1)

        return Response(
            stream_with_context(generate()),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    def __stream_odometry(self) -> Response:
        def generate():
            while True:
                if self.__odometry is not None:
                    json_str = json.dumps(self.__odometry)
                    yield f"data: {json_str}\n\n"
                time.sleep(0.1)

        return Response(stream_with_context(generate()), mimetype="text/event-stream")

    """ BOILERPLATE """

    def __index(self) -> str:
        PATH = Path(__file__).parent / "index.html"
        return open(str(PATH)).read()

    def run_app(self) -> None:
        if self.__thread is not None:
            print("warning: server thread already running, restarting")
            self.__thread.join()
        print("!! MASLAB streamer is running !!")
        self.__thread = Thread(target=lambda: self.__app.run(host="0.0.0.0", port=5000))
        self.__thread.daemon = True
        self.__thread.start()

    def stop_app(self) -> None:
        if self.__thread is not None:
            print("!! MASLAB streamer is stopping !!")
            self.__thread.join()
        else:
            print("warning: no server thread to stop")
