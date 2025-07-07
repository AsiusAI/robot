from typing import Any
import json
from modules.camera import create_local_tracks, stop_camera
from modules.wheels import (
    get_status,
    start_odrive,
    cleanup_motors,
    move,
)
import asyncio
from aiohttp import web
from aiortc import (
    RTCPeerConnection,
    RTCSessionDescription,
    RTCDataChannel,
)


start_odrive()

routes = web.RouteTableDef()


@routes.get("/")
async def index(req: web.Request):
    return web.Response(content_type="text/html", text=open("pages/index.html").read())


@routes.get("/vr")
async def vr(req: web.Request):
    return web.Response(content_type="text/html", text=open("pages/vr.html").read())


@routes.get("/status")
async def status(req: web.Request):
    return web.Response(content_type="application/json", text=json.dumps(get_status()))


@routes.post("/control")
async def control(req: web.Request):
    data = await req.json()
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

    return web.Response(
        content_type="application/json",
        body=json.dumps({"left": left, "right": right, "speed": speed}),
    )


pcs = set()
data_channels: dict[RTCPeerConnection, RTCDataChannel] = {}


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

    audio, video = create_local_tracks()

    if audio:
        pc.addTrack(audio)

    if video:
        pc.addTrack(video)

    await pc.setRemoteDescription(offer)

    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.Response(
        content_type="application/json",
        text=json.dumps(
            {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
        ),
    )


async def on_shutdown(app: Any) -> None:
    cleanup_motors()
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()

    stop_camera()


if __name__ == "__main__":
    app = web.Application()
    app.on_shutdown.append(on_shutdown)
    app.add_routes(routes)
    web.run_app(app, port=8000)
