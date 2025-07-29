import numpy as np
from av import VideoFrame
from aiortc import MediaStreamTrack
import pybullet as p


class BulletCameraTrack(MediaStreamTrack):
  kind = 'video'

  def __init__(self):
    super().__init__()  # required by MediaStreamTrack
    self.width = 640
    self.height = 480
    self.fov = 60
    self.aspect = self.width / self.height
    self.near = 0.1
    self.far = 10.0

  async def recv(self):
    pts, time_base = await self.next_timestamp()

    cam_target = [0.0, 0.0, 0.5]
    cam_distance = 1.0
    cam_yaw = 90
    cam_pitch = -30
    cam_roll = 0

    view_matrix = p.computeViewMatrixFromYawPitchRoll(
      cameraTargetPosition=cam_target,
      distance=cam_distance,
      yaw=cam_yaw,
      pitch=cam_pitch,
      roll=cam_roll,
      upAxisIndex=2,
    )

    projection_matrix = p.computeProjectionMatrixFOV(
      fov=self.fov, aspect=self.aspect, nearVal=self.near, farVal=self.far
    )

    img_arr = p.getCameraImage(
      self.width,
      self.height,
      view_matrix,
      projection_matrix,
      renderer=p.ER_BULLET_HARDWARE_OPENGL,
    )

    rgb = np.reshape(img_arr[2], (self.height, self.width, 4))[:, :, :3]  # strip alpha
    frame = VideoFrame.from_ndarray(rgb, format='rgb24')
    frame.pts = pts
    frame.time_base = time_base
    return frame
