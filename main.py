from flask import Flask, Response, render_template_string, request, jsonify
import signal
import sys
import atexit
from modules.camera import generate_frames
from modules.wheels import (
    get_status,
    start_odrive,
    cleanup_motors,
    move,
)

start_odrive()

app = Flask(__name__)


def signal_handler(sig, frame):
    """Handle Ctrl+C and other termination signals"""
    print("\nShutting down gracefully...")
    cleanup_motors()
    sys.exit(0)


# Register signal handlers and cleanup
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)
atexit.register(cleanup_motors)


@app.route("/")
def index():
    return render_template_string(open("pages/index.html", "r").read())


@app.route("/vr")
def vr():
    return render_template_string(open("pages/vr.html", "r").read())


@app.route("/video_feed")
def video_feed():
    return Response(
        generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/status")
def status():
    return get_status()


@app.route("/control", methods=["POST"])
def control():
    data = request.get_json()
    x = data.get("x", 0)  # horizontal (-1 to 1)
    y = data.get("y", 0)  # vertical (-1 to 1)
    speed = data.get("speed", 1.0)  # speed multiplier (0.1 to 3.0)

    # Map joystick input to differential drive wheel speeds
    left = y + x
    right = y - x

    # Clamp to [-1, 1]
    left = max(-1, min(1, left))
    right = max(-1, min(1, right))

    move(left=left, right=right, speed=speed)
    print(f"{left=}, {right=}, speed={speed}")

    return jsonify(left=left, right=right, speed=speed)


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8000, debug=False)
