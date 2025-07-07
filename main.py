from typing import Any
import json
from modules.arms import connect_arms, disconnect_arms, set_gripper_pos
from modules.camera import create_local_tracks, stop_camera
from modules.wheels import (
    control,
    get_status,
    start_odrive,
    cleanup_motors,
)
import asyncio
from aiohttp import web
from aiortc import (
    RTCPeerConnection,
    RTCSessionDescription,
    RTCDataChannel,
)


start_odrive()
connect_arms()

routes = web.RouteTableDef()


pcs: set[RTCPeerConnection] = set()
data_channels: dict[RTCPeerConnection, RTCDataChannel] = {}


def send_message(pc, type, data):
    if pc not in data_channels:
        return
    channel = data_channels[pc]
    channel.send(json.dumps({"type": type, "data": data}))


def send_to_all(type, data):
    print(f"Sending {type} {data} to {len(pcs)} clients")
    for pc in pcs:
        send_message(pc, type, data)


async def status_monitor():
    while True:
        voltage, current, uptime = get_status()
        data = {"voltage": voltage, "current": current, "uptime": uptime}
        send_to_all("status", data)
        await asyncio.sleep(5)


@routes.get("/")
async def index(req: web.Request):
    return web.Response(content_type="text/html", text=open("pages/index.html").read())


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
            data_channels.pop(pc)

    @pc.on("datachannel")
    def on_datachannel(channel: RTCDataChannel) -> None:
        print(f"Data channel received: {channel.label}")
        data_channels[pc] = channel

        @channel.on("close")
        def on_close() -> None:
            print(f"Data channel '{channel.label}' closed")
            data_channels.pop(pc)
            pcs.discard(pc)

        @channel.on("message")
        def on_message(message) -> None:
            msg = json.loads(message)
            try:
                data = msg["data"]

                if msg["type"] == "control":
                    control(x=data["x"], y=data["y"], speed=data["speed"])
                if msg["type"] == "vr":
                    joystick = None
                    if "left" in data:
                        left = data["left"]
                        joystick = left["joystick"]
                        set_gripper_pos("left", (left["trigger"] - 1) * -1)

                    if "right" in data:
                        right = data["right"]
                        joystick = right["joystick"]
                        set_gripper_pos("right", (right["trigger"] - 1) * -1)

                    if joystick:
                        control(
                            x=joystick["x"],
                            y=joystick["y"] * -1,
                            speed=0.4,
                        )
            except Exception as e:
                print(e)

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
    data_channels.clear()

    stop_camera()
    disconnect_arms()


if __name__ == "__main__":
    app = web.Application()
    app.on_shutdown.append(on_shutdown)
    app.add_routes(routes)

    async def start_status_monitor(application):
        asyncio.create_task(status_monitor())

    app.on_startup.append(start_status_monitor)
    web.run_app(app, port=8000)
