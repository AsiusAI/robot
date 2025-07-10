import dataclasses
import math
from typing import Any
import json
from modules.camera import create_local_tracks, stop_camera
import asyncio
from aiohttp import web
from aiortc import (
    RTCPeerConnection,
    RTCSessionDescription,
    RTCDataChannel,
)

from robots.v1 import RobotV1

robot = RobotV1()
robot.start()

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
        status = robot.status()
        send_to_all("status", dataclasses.asdict(status))
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
                    robot.move_with_joystick(
                        x=data["x"], y=data["y"], speed=data["speed"]
                    )
                if msg["type"] == "vr":
                    joystick = None
                    if "left" in data:
                        controller = data["left"]
                        joystick = controller["joystick"]

                    if "right" in data:
                        controller = data["right"]
                        joystick = controller["joystick"]
                        if controller["pos"]:
                            pos = controller["pos"]
                            rot = controller["rot"]
                            res = robot._get_ik(
                                [pos["x"], pos["y"], pos["z"]],
                                [rot["x"], rot["y"], rot["z"], rot["w"]],
                            )
                            res.gripper = (controller["trigger"] - 1) * -(math.pi / 2)
                            robot.move_arm("right", res)

                    if joystick:
                        robot.move_with_joystick(
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
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()
    data_channels.clear()
    robot.stop()
    stop_camera()


if __name__ == "__main__":
    app = web.Application()
    app.on_shutdown.append(on_shutdown)
    app.add_routes(routes)

    async def start_status_monitor(application):
        asyncio.create_task(status_monitor())

    app.on_startup.append(start_status_monitor)
    web.run_app(app, port=8000)
