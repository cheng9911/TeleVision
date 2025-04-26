import numpy as np
import cv2
import time
import pyrealsense2 as rs
from constants_vuer import *
from TeleVision import OpenTeleVision
from pytransform3d import rotations
from multiprocessing import shared_memory, Queue, Event

# ------------------------ RealSense 初始化 ------------------------
resolution = (720, 1280)
crop_size_w = 1
crop_size_h = 0
resolution_cropped = (resolution[0] - crop_size_h, resolution[1] - 2 * crop_size_w)

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, resolution[1], resolution[0], rs.format.bgr8, 30)

pipeline.start(config)


# ------------------------ 共享内存初始化（单路图像） ------------------------
img_shape = (resolution_cropped[0], resolution_cropped[1], 3)  # ✅ 单摄像头尺寸
img_height, img_width = resolution_cropped[:2]  # 450 * 600
shm = shared_memory.SharedMemory(create=True, size=np.prod(img_shape) * np.uint8().itemsize)
shm_name = shm.name
img_array = np.ndarray((img_shape[0], img_shape[1], 3), dtype=np.uint8, buffer=shm.buf)
image_queue = Queue()
toggle_streaming = Event()
tv = OpenTeleVision(resolution_cropped, shm.name, image_queue, toggle_streaming,ngrok=True)

# ------------------------ 主循环 ------------------------
try:
    while True:
        start = time.time()

        # 获取头部姿态矩阵并转为欧拉角
        head_mat = grd_yup2grd_zup[:3, :3] @ tv.head_matrix[:3, :3] @ grd_yup2grd_zup[:3, :3].T
        if np.sum(head_mat) == 0:
            head_mat = np.eye(3)
        head_quat = rotations.quaternion_from_matrix(head_mat)
        ypr = rotations.euler_from_quaternion(head_quat, 2, 1, 0, False)  # Yaw, Pitch, Roll

        # print("当前手腕姿态（YPR）：", ypr)  # 输出欧拉角

        # 获取图像帧
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())

        # 裁剪并转换颜色空间
        bgr_cropped = color_image[crop_size_h:, crop_size_w:-crop_size_w]
        rgb = cv2.cvtColor(bgr_cropped, cv2.COLOR_BGR2RGB)

        np.copyto(img_array, rgb)

        end = time.time()
        # print("FPS:", 1 / (end - start))

except KeyboardInterrupt:
    print("停止运行")
finally:
    pipeline.stop()
    shm.close()
    shm.unlink()