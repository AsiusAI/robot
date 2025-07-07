import argparse
import asyncio
import json
import logging
import os
import platform
import ssl
from typing import Optional

from aiohttp import web
from aiortc import (
    MediaStreamTrack,
    RTCPeerConnection,
    RTCRtpSender,
    RTCSessionDescription,
    RTCDataChannel,
)
from aiortc.contrib.media import MediaPlayer, MediaRelay

routes = web.RouteTableDef()

pcs = set()
data_channels: dict[RTCPeerConnection, RTCDataChannel] = {}
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

@routes.get("/")
async def index(request: web.Request):
    return web.Response(content_type="text/html", text=open("webrtc.html", "r").read())


def send_message(pc, type, data):
    channel = data_channels[pc]
    channel.send(json.dumps({"type": type, "data": data}))

@routes.post("/offer")
async def offer(request: web.Request) -> web.Response:
    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

    pc = RTCPeerConnection()
    pcs.add(pc)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange() -> None:
        print("Connection state is %s" % pc.connectionState)
        if pc.connectionState == "failed":
            await pc.close()
            pcs.discard(pc)

    @pc.on("datachannel")
    def on_datachannel(channel: RTCDataChannel) -> None:
        print(f"Data channel received: {channel.label}")

        @channel.on("open")
        def on_open() -> None:
            print(f"Data channel '{channel.label}' opened")
            data_channels[pc] = channel

        @channel.on("close")
        def on_close() -> None:
            print(f"Data channel '{channel.label}' closed")
            del data_channels[pc]

        @channel.on("message")
        def on_message(message) -> None:
            print(f"Received message on channel '{channel.label}': {message}")
            data = json.loads(message)
            print(f"Parsed message: {data}")

    # open media source
    audio, video = create_local_tracks()

    if audio:
        audio_sender = pc.addTrack(audio)

    if video:
        video_sender = pc.addTrack(video)

    await pc.setRemoteDescription(offer)

    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.Response(
        content_type="application/json",
        text=json.dumps(
            {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
        ),
    )


async def on_shutdown(app: web.Application) -> None:
    # Close peer connections.
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()

    # If a shared webcam was opened, stop it.
    if webcam is not None and webcam.video is not None:
        webcam.video.stop()


if __name__ == "__main__":
    app = web.Application()
    app.on_shutdown.append(on_shutdown)
    app.add_routes(routes)
    web.run_app(app, port=8000)
