from modules.wheels import (
    get_status,
    start_odrive,
    cleanup_motors,
    move,
)
import platform
from typing import Optional

from aiortc import (
    MediaStreamTrack,
)
from aiortc.contrib.media import MediaPlayer, MediaRelay


relay = None
webcam = None


def create_local_tracks() -> (
    tuple[Optional[MediaStreamTrack], Optional[MediaStreamTrack]]
):
    global relay, webcam

    options = {"framerate": "30", "video_size": "640x480"}
    if relay is None:
        if platform.system() == "Darwin":
            webcam = MediaPlayer("default:none", format="avfoundation", options=options)
        elif platform.system() == "Windows":
            webcam = MediaPlayer(
                "video=Integrated Camera", format="dshow", options=options
            )
        else:
            webcam = MediaPlayer("/dev/video0", format="v4l2", options=options)
        relay = MediaRelay()

    if webcam and webcam.video:
        return None, relay.subscribe(webcam.video)
    return None, None


def stop_camera():
    if webcam is not None and webcam.video is not None:
        webcam.video.stop()
