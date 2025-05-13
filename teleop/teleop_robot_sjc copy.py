import numpy as np
import cv2
import time
import pyrealsense2 as rs
from constants_vuer import *
from TeleVision import OpenTeleVision
from pytransform3d import rotations
from multiprocessing import shared_memory, Queue, Event
import math
import numpy as np
import torch

from TeleVision_sjc import OpenTeleVision
from Preprocessor import VuerPreprocessor
from constants_vuer import tip_indices
from dex_retargeting.retargeting_config import RetargetingConfig
from pytransform3d import rotations

from pathlib import Path
import argparse
import time
import yaml
import  socket

from multiprocessing import Array, Process, shared_memory, Queue, Manager, Event, Semaphore
class VuerTeleop:
    def __init__(self, config_file_path):
        self.resolution = (720, 1280)
        self.crop_size_w = 0
        self.crop_size_h = 0
        self.resolution_cropped = (self.resolution[0]-self.crop_size_h, self.resolution[1]-2*self.crop_size_w)

        self.img_shape = (self.resolution_cropped[0], self.resolution_cropped[1], 3)
        self.img_height, self.img_width = self.resolution_cropped[:2]

        self.shm = shared_memory.SharedMemory(create=True, size=np.prod(self.img_shape) * np.uint8().itemsize)
        self.img_array = np.ndarray((self.img_shape[0], self.img_shape[1], 3), dtype=np.uint8, buffer=self.shm.buf)
        image_queue = Queue()
        toggle_streaming = Event()
        self.tv = OpenTeleVision(self.resolution_cropped, self.shm.name, image_queue, toggle_streaming,ngrok=True)
        self.processor = VuerPreprocessor()

        RetargetingConfig.set_default_urdf_dir('../assets')
        with Path(config_file_path).open('r') as f:
            cfg = yaml.safe_load(f)
        left_retargeting_config = RetargetingConfig.from_dict(cfg['left'])
        right_retargeting_config = RetargetingConfig.from_dict(cfg['right'])
        self.left_retargeting = left_retargeting_config.build()
        self.right_retargeting = right_retargeting_config.build()

    def step(self):
        head_mat, left_wrist_mat, right_wrist_mat, left_hand_mat, right_hand_mat = self.processor.process(self.tv)

        head_rmat = head_mat[:3, :3]

        left_pose = np.concatenate([left_wrist_mat[:3, 3] + np.array([-0.6, 0, 1.6]),
                                    rotations.quaternion_from_matrix(left_wrist_mat[:3, :3])[[1, 2, 3, 0]]])
        right_pose = np.concatenate([right_wrist_mat[:3, 3] + np.array([-0.6, 0, 1.6]),
                                     rotations.quaternion_from_matrix(right_wrist_mat[:3, :3])[[1, 2, 3, 0]]])
        left_qpos = self.left_retargeting.retarget(left_hand_mat[tip_indices])[[4, 5, 6, 7, 10, 11, 8, 9, 0, 1, 2, 3]]
        right_qpos = self.right_retargeting.retarget(right_hand_mat[tip_indices])[[4, 5, 6, 7, 10, 11, 8, 9, 0, 1, 2, 3]]

        return head_rmat, left_pose, right_pose, left_qpos, right_qpos
if __name__ == '__main__':
    # ------------------------ RealSense 初始化 ------------------------
    resolution = (720, 1280)
    crop_size_w = 0
    crop_size_h = 0
    resolution_cropped = (resolution[0] - crop_size_h, resolution[1] - 2 * crop_size_w)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, resolution[1], resolution[0], rs.format.bgr8, 30)

    pipeline.start(config)


    teleoperator = VuerTeleop('inspire_hand.yml')
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(("0.0.0.0", 12345))
    server_socket.listen(1)
    print("[*] 等待客户端连接...")
    # server_socket.setblocking(False)  # 设置非阻塞模式
    client_socket = None
    client_socket, addr = server_socket.accept()


    try:
        while True:
            head_rmat, left_pose, right_pose, left_qpos, right_qpos = teleoperator.step()
            # print("left_qpos",left_qpos)
            # print("right_qpos",right_qpos)
            try:
                if client_socket is not None:
                    # 取8 0 2 4 6 10
                    com_right_qpos = right_qpos[[8, 0, 2, 4, 6, 10]].tolist()
                    # com_right_qpos=right_qpos[:6].tolist()
                    data_str = ','.join(map("{:.4f}".format, com_right_qpos)) + '\n'
                    client_socket.sendall(data_str.encode("utf-8"))
                    print(f"发送数据: {data_str.strip()}")
                else:
                    pass
            except BrokenPipeError:
                pass

            # np.copyto(teleoperator.img_array, np.hstack((right_img,left_img)))

            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())

            # 裁剪并转换颜色空间
            bgr_cropped = color_image[crop_size_h:, crop_size_w:-crop_size_w]
            if crop_size_w > 0:
                bgr_cropped = color_image[crop_size_h:, crop_size_w:-crop_size_w]
            else:
                bgr_cropped = color_image[crop_size_h:]


            np.copyto(teleoperator.img_array, bgr_cropped)
    except KeyboardInterrupt:
        print("停止运行")
    finally:
        pipeline.stop()
        teleoperator.shm.close()
        teleoperator.shm.unlink()