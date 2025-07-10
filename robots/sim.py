import dataclasses
from typing import Literal, Optional
import numpy as np
from aiortc import MediaStreamTrack
from numpy.typing import NDArray
from robots import ArmPosition, Robot, Status
import pybullet as p
import time
import threading
import pybullet_data
from aiortc import VideoStreamTrack
import av


class SimRobot(Robot):
    def __init__(self):
        self.left_arm_queue: Optional[dict] = None
        self.right_arm_queue: Optional[dict] = None
        self.move_queue: Optional[dict] = None
        self.image_queue: Optional[NDArray] = None
        self.running = True
        self.start_time = time.time()

    def start(self):
        threading.Thread(target=self.run_sim_loop, daemon=True).start()

    def stop(self):
        self.running = False

    def run_sim_loop(self):
        p.connect(p.DIRECT)
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
        frame_counter = 0
        while self.running:
            for queue in [self.left_arm_queue, self.right_arm_queue]:
                if queue == None:
                    continue
                print(queue)
                # the actual sim command
                pos = ArmPosition(**queue["pos"])
                positions = {
                    0: pos.shoulder_pan,
                    1: pos.shoulder_lift,
                    2: pos.elbow_flex,
                    3: pos.wrist_flex,
                    4: pos.wrist_roll,
                    6: pos.gripper,
                }
                positions = {k: v for k, v in positions.items() if v is not None}
                p.setJointMotorControlArray(
                    bodyIndex=arms[queue["arm"]],
                    jointIndices=positions.keys(),
                    controlMode=p.POSITION_CONTROL,
                    targetPositions=positions.values(),
                    forces=[100.0] * len(positions),
                )
            self.left_arm_queue = None
            self.right_arm_queue = None

            if self.move_queue:
                self.move_queue = None
                pass

            p.stepSimulation()
            time.sleep(1.0 / 120.0)
            frame_counter += 1
            if frame_counter % 4 == 0:
                width, height, rgb, _, _ = p.getCameraImage(
                    width=640,
                    height=480,
                    viewMatrix=view_matrix,
                    projectionMatrix=projection_matrix,
                    renderer=p.ER_BULLET_HARDWARE_OPENGL,
                )
                self.image_queue = np.array(rgb).reshape(height, width, 4)[:, :, :3]

    def status(self):
        return Status(0.0, 0.0, int(time.time() - self.start_time))

    def get_media_stream(
        self,
    ) -> tuple[Optional[MediaStreamTrack], Optional[MediaStreamTrack]]:
        return None, SimVideoTrack(self)

    def move(self, left: float, right: float, speed: float):
        self.move_queue = {"left": left, "right": right, "speed": speed}

    def move_arm(self, arm: Literal["left", "right"], pos: ArmPosition):
        if arm == "left":
            self.left_arm_queue = {"arm": arm, "pos": dataclasses.asdict(pos)}
        if arm == "right":
            self.right_arm_queue = {"arm": arm, "pos": dataclasses.asdict(pos)}


class SimVideoTrack(VideoStreamTrack):
    def __init__(self, sim_robot: SimRobot):
        super().__init__()
        self.sim = sim_robot

    async def recv(self):
        try:
            img = self.sim.image_queue
            frame = av.VideoFrame.from_ndarray(np.array(img, np.uint8), format="rgb24")
            frame.pts, frame.time_base = await self.next_timestamp()
            return frame
        except Exception as e:
            print(e)
            return []
