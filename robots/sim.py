import dataclasses
from typing import Literal, Optional
import numpy as np
from aiortc import MediaStreamTrack
from robots import ArmPosition, Robot, Status
import pybullet as p
import time
from queue import Queue
import threading
import pybullet_data
from aiortc import VideoStreamTrack
import av


class SimRobot(Robot):
    def __init__(self):
        self.command_queue = Queue()
        self.image_queue = Queue()
        self.running = True
        self.start_time = time.time()

    def start(self):
        threading.Thread(target=self.run_sim_loop, daemon=True).start()

    def stop(self):
        self.running = False

    def run_sim_loop(self):
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")

        arms = {
            "left": p.loadURDF(
                "sim/SO101/so101_new_calib.urdf",
                basePosition=[0.0, 0.08, 0.5],
                baseOrientation=p.getQuaternionFromEuler([-np.pi / 2, 0, 0]),
                useFixedBase=True,
            ),
            "right": p.loadURDF(
                "sim/SO101/so101_new_calib.urdf",
                basePosition=[0.0, 0.0, 0.5],
                baseOrientation=p.getQuaternionFromEuler([np.pi / 2, 0, 0]),
                useFixedBase=True,
            ),
        }

        while self.running:
            try:
                cmd = self.command_queue.get_nowait()
                if cmd["type"] == "move_arm":
                    # the actual sim command
                    arm_joints = [0, 1, 2, 3, 4, 6]
                    pos = ArmPosition(**cmd["pos"])
                    p.setJointMotorControlArray(
                        bodyIndex=arms[cmd["arm"]],
                        jointIndices=arm_joints,
                        controlMode=p.POSITION_CONTROL,
                        targetPositions=[
                            pos.shoulder_pan,
                            pos.shoulder_lift,
                            pos.elbow_flex,
                            pos.wrist_flex,
                            pos.wrist_roll,
                            pos.gripper,
                        ],
                        forces=[100.0] * len(arm_joints),
                    )
                    pass
                if cmd["type"] == "move":
                    print("Not implemented!")
                    pass
            except:
                pass

            p.stepSimulation()
            time.sleep(1.0 / 240.0)

            view_matrix = p.computeViewMatrixFromYawPitchRoll(
                cameraTargetPosition=[0, 0.04, 0.5],
                distance=1,
                yaw=-90,
                pitch=-20,
                roll=0,
                upAxisIndex=2,
            )

            projection_matrix = p.computeProjectionMatrixFOV(
                fov=60,
                aspect=640 / 480,
                nearVal=0.1,
                farVal=10.0,
            )

            width, height, rgb, _, _ = p.getCameraImage(
                width=640,
                height=480,
                viewMatrix=view_matrix,
                projectionMatrix=projection_matrix,
                renderer=p.ER_BULLET_HARDWARE_OPENGL,
            )

            self.image_queue.put(np.array(rgb).reshape(height, width, 4)[:, :, :3])

    def send_command(self, command: dict):
        self.command_queue.put(command)

    def get_latest_image(self):
        if not self.image_queue.empty():
            return self.image_queue.get()
        return None

    def status(self):
        return Status(0.0, 0.0, int(time.time() - self.start_time))

    def get_media_stream(
        self,
    ) -> tuple[Optional[MediaStreamTrack], Optional[MediaStreamTrack]]:
        return None, SimVideoTrack(self)

    def move(self, left: float, right: float, speed: float):
        self.send_command(
            {"type": "move", "left": left, "right": right, "speed": speed}
        )

    def move_arm(self, arm: Literal["left", "right"], pos: ArmPosition):
        self.send_command(
            {"type": "move_arm", "arm": arm, "pos": dataclasses.asdict(pos)}
        )


class SimVideoTrack(VideoStreamTrack):
    def __init__(self, sim_robot: SimRobot):
        super().__init__()
        self.sim = sim_robot

    async def recv(self):
        try:
            img = self.sim.get_latest_image()
            frame = av.VideoFrame.from_ndarray(np.array(img, np.uint8), format="rgb24")
            frame.pts, frame.time_base = await self.next_timestamp()
            return frame
        except Exception as e:
            print(e)
            return []
