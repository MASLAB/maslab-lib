from typing import Dict, TypedDict
import time
import json
from threading import Thread
from pathlib import Path

from flask import Flask, stream_with_context, Response
import cv2


class Odometry(TypedDict):
    x: float
    y: float
    theta: float
    circles: list[Dict]
    lines: list[Dict]

class Streamer:
    def __init__(self) -> None:
        self.json_data = {}
        self.img = None
        self.odometry = {
            "x": 0,
            "y": 0,
            "theta": 0,
        }

        self.app = Flask(__name__)
        self.app.add_url_rule("/stream_img", "stream_img", self.stream_img)
        self.app.add_url_rule("/stream_data", "stream_data", self.stream_data)
        self.app.add_url_rule(
            "/stream_odometry", "stream_odometry", self.stream_odometry
        )
        self.app.add_url_rule("/", "index", self.index)

        self.run_app()

    """ SETTERS """

    def set_data(self, data: Dict) -> None:
        self.json_data = data

    def set_img(self, img: cv2.Mat) -> None:
        self.img = img

    def set_odometry(self, odometry: Odometry) -> None:
        self.odometry = odometry

    """ STREAMING ENDPOINTS """

    def stream_data(self) -> Response:
        def generate():
            while True:
                json_str = json.dumps(self.json_data)
                yield f"data: {json_str}\n\n"
                time.sleep(0.1)

        return Response(stream_with_context(generate()), mimetype="text/event-stream")

    def stream_img(self) -> Response:
        def generate():
            while True:
                if self.img is not None:
                    _, img_encoded = cv2.imencode(".jpg", self.img)
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

    def stream_odometry(self) -> Response:
        def generate():
            while True:
                if self.odometry is not None:
                    json_str = json.dumps(self.odometry)
                    yield f"data: {json_str}\n\n"
                time.sleep(0.1)

        return Response(stream_with_context(generate()), mimetype="text/event-stream")

    """ BOILERPLATE """

    def index(self) -> str:
        # get actual path of index.html
        PATH = Path(__file__).parent / "index.html"
        return open(str(PATH)).read()

    def run_app(self) -> None:
        print("!! MASLAB streamer is running !!")
        self.thread = Thread(target=lambda: self.app.run(host="0.0.0.0", port=5000))
        self.thread.daemon = True
        self.thread.start()
