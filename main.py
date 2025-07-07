from typing import Any
import json
from modules.camera import generate_frames
from modules.wheels import (
    get_status,
    start_odrive,
    cleanup_motors,
    move,
)
from aiohttp import web


start_odrive()

routes = web.RouteTableDef()


@routes.get("/")
async def index(req: web.Request):
    return web.Response(content_type="text/html", text=open("pages/index.html").read())


@routes.get("/vr")
async def vr(req: web.Request):
    return web.Response(content_type="text/html", text=open("pages/vr.html").read())


@routes.get("/video_feed")
async def video_feed(req: web.Request):
    return web.Response(
        content_type="multipart/x-mixed-replace; boundary=frame", body=generate_frames()
    )


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

    return web.Response(content_type="application/json", body= json.dumps({"left": left, "right": right, "speed": speed}))


async def on_shutdown(app: Any) -> None:
    cleanup_motors()


if __name__ == "__main__":
    app = web.Application()
    app.on_shutdown.append(on_shutdown)
    app.add_routes(routes)
    web.run_app(app, port=8000)
