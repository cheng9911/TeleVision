from isaacgym import gymapi
from isaacgym import gymutil
from isaacgym import gymtorch

import math
import numpy as np
import torch

from TeleVision import OpenTeleVision
from Preprocessor import VuerPreprocessor
from constants_vuer import tip_indices
from dex_retargeting.retargeting_config import RetargetingConfig
from pytransform3d import rotations
import socket
from pathlib import Path
import argparse
import time
import yaml
from multiprocessing import Array, Process, shared_memory, Queue, Manager, Event, Semaphore
# def apply_dynamic_offset(qpos, offset, joint_min_values, joint_max_values):
#     target_indices = np.arange(6)  # 前6个关节校准
#     current_range_ratio = np.abs(qpos[target_indices] - offset) / (joint_max_values - offset)
#     # 限制current_range_ratio在0-1之间
#     current_range_ratio=np.clip(current_range_ratio,0,1)
#     dynamic_offset = offset * (1 - current_range_ratio)  # 线性动态偏移
#     qpos[target_indices] -= dynamic_offset
#     # qpos[target_indices] = np.clip(qpos[target_indices], joint_min_values, joint_max_values)
#     return qpos
def apply_dynamic_offset(qpos, offset, joint_min_values, joint_max_values):
    target_indices = np.arange(6)  # 前6个关节进行偏移校准

    # 计算每个关节当前归一化位置（范围在 0~1）
    normalized_qpos = (qpos[target_indices] - joint_min_values) / (joint_max_values - joint_min_values)

    # 设定一个“修正区间”：只在张开状态（小于阈值）下进行偏移
    threshold = 0.4  # 在归一化范围内，小于0.4的视为“张开”
    correction_strength = 1.0 - normalized_qpos / threshold
    correction_strength = np.clip(correction_strength, 0.0, 1.0)  # 只在小角度下有偏移

    # 计算动态偏移：越接近关节张开（小角度），越强；越弯曲，偏移趋近为 0
    dynamic_offset = offset * correction_strength

    # 应用偏移并确保不越界
    qpos[target_indices] -= dynamic_offset
    qpos[target_indices] = np.clip(qpos[target_indices], joint_min_values, joint_max_values)

    return qpos

class VuerTeleop:
    def __init__(self, config_file_path):
        self.resolution = (720, 1280)
        self.crop_size_w = 0
        self.crop_size_h = 0
        self.resolution_cropped = (self.resolution[0]-self.crop_size_h, self.resolution[1]-2*self.crop_size_w)

        self.img_shape = (self.resolution_cropped[0], 2 * self.resolution_cropped[1], 3)
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
        # left_qpos = self.left_retargeting.retarget(left_hand_mat[tip_indices])[[4, 5, 6, 7, 10, 11, 8, 9, 0, 1, 2, 3]]
        #
        # right_qpos = self.right_retargeting.retarget(right_hand_mat[tip_indices])[[4, 5, 6, 7, 10, 11, 8, 9, 0, 1, 2, 3]]
        # 示例 offset：每个手都可能不同，你可以自定义
        # offset = np.array([ 3.66684637e-19, -4.54499982e-02 , 4.94151353e-19 ,-4.54499982e-02,4.54999268e-19,3.41993263e-01])
        offset_left = np.array([8.38016631e-02, -4.52312553e-02,  4.56259977e-11, -4.54499981e-02,0, -4.54499982e-02])
        offset_right = np.array([8.38016631e-02, -4.52312553e-02, 4.56259977e-11, -4.54499981e-02, 0, -4.54499982e-02])
        joint_min_values = np.array([0, 0, 0, 0, 0, 0])  # 示例：六轴关节范围
        joint_max_values = np.array([1.308, 1.47, 1.47, 1.47, 1.7, 0.6])
        # print("joint names:", self.left_retargeting.optimizer.target_joint_names)

        # Retarget 并重新排列关节顺序
        # left_qpos = self.left_retargeting.retarget(left_hand_mat[tip_indices])[[4, 5, 6, 7, 10, 11, 8, 9, 0, 1, 2, 3]]
        # right_qpos = self.right_retargeting.retarget(right_hand_mat[tip_indices])[
        #     [4, 5, 6, 7, 10, 11, 8, 9, 0, 1, 2, 3]]
        # reorder_indices = [0, 4, 6, 8, 10, 1, 5, 7, 9, 11, 2, 3]
        reorder_indices = [4, 5, 6, 7, 10, 11, 8, 9, 0, 1, 2, 3]


        left_qpos = self.left_retargeting.retarget(left_hand_mat[tip_indices])[reorder_indices]
        right_qpos = self.right_retargeting.retarget(right_hand_mat[tip_indices])[reorder_indices]
        # print("left_qpos",left_qpos)
        # print("right_qpos",right_qpos)
        # # 分别对前6个关节应用动态偏移
        # left_qpos = apply_dynamic_offset(left_qpos, offset_left, joint_min_values, joint_max_values)
        # right_qpos = apply_dynamic_offset(right_qpos, offset_right, joint_min_values, joint_max_values)


        return head_rmat, left_pose, right_pose, left_qpos, right_qpos

class Sim:
    def __init__(self,
                 print_freq=False):
        self.print_freq = print_freq

        # initialize gym
        self.gym = gymapi.acquire_gym()

        # configure sim
        sim_params = gymapi.SimParams()
        sim_params.dt = 1 / 60
        sim_params.substeps = 2
        sim_params.up_axis = gymapi.UP_AXIS_Z
        sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.81)
        sim_params.physx.solver_type = 1
        sim_params.physx.num_position_iterations = 4
        sim_params.physx.num_velocity_iterations = 1
        sim_params.physx.max_gpu_contact_pairs = 8388608
        sim_params.physx.contact_offset = 0.002
        sim_params.physx.friction_offset_threshold = 0.001
        sim_params.physx.friction_correlation_distance = 0.0005
        sim_params.physx.rest_offset = 0.0
        sim_params.physx.use_gpu = True
        sim_params.use_gpu_pipeline = False

        self.sim = self.gym.create_sim(0, 0, gymapi.SIM_PHYSX, sim_params)
        if self.sim is None:
            print("*** Failed to create sim")
            quit()

        plane_params = gymapi.PlaneParams()
        plane_params.distance = 0.0
        plane_params.normal = gymapi.Vec3(0.0, 0.0, 1.0)
        self.gym.add_ground(self.sim, plane_params)

        # load table asset
        table_asset_options = gymapi.AssetOptions()
        table_asset_options.disable_gravity = True
        table_asset_options.fix_base_link = True
        table_asset = self.gym.create_box(self.sim, 0.8, 0.8, 0.1, table_asset_options)

        # load cube asset
        cube_asset_options = gymapi.AssetOptions()
        cube_asset_options.density = 10
        cube_asset = self.gym.create_box(self.sim, 0.05, 0.05, 0.05, cube_asset_options)

        asset_root = "../assets"
        left_asset_path = "inspire_hand/inspire_hand_left.urdf"
        right_asset_path = "inspire_hand/inspire_hand_right.urdf"
        asset_options = gymapi.AssetOptions()
        asset_options.fix_base_link = True
        asset_options.default_dof_drive_mode = gymapi.DOF_MODE_POS
        left_asset = self.gym.load_asset(self.sim, asset_root, left_asset_path, asset_options)
        right_asset = self.gym.load_asset(self.sim, asset_root, right_asset_path, asset_options)
        self.dof = self.gym.get_asset_dof_count(left_asset)

        # set up the env grid
        num_envs = 1
        num_per_row = int(math.sqrt(num_envs))
        env_spacing = 1.25
        env_lower = gymapi.Vec3(-env_spacing, 0.0, -env_spacing)
        env_upper = gymapi.Vec3(env_spacing, env_spacing, env_spacing)
        np.random.seed(0)
        self.env = self.gym.create_env(self.sim, env_lower, env_upper, num_per_row)

        # table
        pose = gymapi.Transform()
        pose.p = gymapi.Vec3(0, 0, 1.2)
        pose.r = gymapi.Quat(0, 0, 0, 1)
        table_handle = self.gym.create_actor(self.env, table_asset, pose, 'table', 0)
        color = gymapi.Vec3(0.5, 0.5, 0.5)
        self.gym.set_rigid_body_color(self.env, table_handle, 0, gymapi.MESH_VISUAL_AND_COLLISION, color)

        # cube
        pose = gymapi.Transform()
        pose.p = gymapi.Vec3(0, 0, 1.25)
        pose.r = gymapi.Quat(0, 0, 0, 1)
        cube_handle = self.gym.create_actor(self.env, cube_asset, pose, 'cube', 0)
        color = gymapi.Vec3(1, 0.5, 0.5)
        self.gym.set_rigid_body_color(self.env, cube_handle, 0, gymapi.MESH_VISUAL_AND_COLLISION, color)

        # left_hand
        pose = gymapi.Transform()
        pose.p = gymapi.Vec3(-0.6, 0, 1.6)
        pose.r = gymapi.Quat(0, 0, 0, 1)
        self.left_handle = self.gym.create_actor(self.env, left_asset, pose, 'left', 1, 1)
        self.gym.set_actor_dof_states(self.env, self.left_handle, np.zeros(self.dof, gymapi.DofState.dtype),
                                      gymapi.STATE_ALL)
        left_idx = self.gym.get_actor_index(self.env, self.left_handle, gymapi.DOMAIN_SIM)

        # right_hand
        pose = gymapi.Transform()
        pose.p = gymapi.Vec3(-0.6, 0, 1.6)
        pose.r = gymapi.Quat(0, 0, 0, 1)
        self.right_handle = self.gym.create_actor(self.env, right_asset, pose, 'right', 1, 1)
        self.gym.set_actor_dof_states(self.env, self.right_handle, np.zeros(self.dof, gymapi.DofState.dtype),
                                      gymapi.STATE_ALL)
        right_idx = self.gym.get_actor_index(self.env, self.right_handle, gymapi.DOMAIN_SIM)

        self.root_state_tensor = self.gym.acquire_actor_root_state_tensor(self.sim)
        self.gym.refresh_actor_root_state_tensor(self.sim)
        self.root_states = gymtorch.wrap_tensor(self.root_state_tensor)
        self.left_root_states = self.root_states[left_idx]
        self.right_root_states = self.root_states[right_idx]

        # create default viewer
        self.viewer = self.gym.create_viewer(self.sim, gymapi.CameraProperties())
        if self.viewer is None:
            print("*** Failed to create viewer")
            quit()
        cam_pos = gymapi.Vec3(1, 1, 2)
        cam_target = gymapi.Vec3(0, 0, 1)
        self.gym.viewer_camera_look_at(self.viewer, None, cam_pos, cam_target)

        self.cam_lookat_offset = np.array([1, 0, 0])
        self.left_cam_offset = np.array([0, 0.033, 0])
        self.right_cam_offset = np.array([0, -0.033, 0])
        self.cam_pos = np.array([-0.6, 0, 1.6])

        # create left 1st preson viewer
        camera_props = gymapi.CameraProperties()
        camera_props.width = 1280
        camera_props.height = 720
        self.left_camera_handle = self.gym.create_camera_sensor(self.env, camera_props)
        self.gym.set_camera_location(self.left_camera_handle,
                                     self.env,
                                     gymapi.Vec3(*(self.cam_pos + self.left_cam_offset)),
                                     gymapi.Vec3(*(self.cam_pos + self.left_cam_offset + self.cam_lookat_offset)))

        # create right 1st preson viewer
        camera_props = gymapi.CameraProperties()
        camera_props.width = 1280
        camera_props.height = 720
        self.right_camera_handle = self.gym.create_camera_sensor(self.env, camera_props)
        self.gym.set_camera_location(self.right_camera_handle,
                                     self.env,
                                     gymapi.Vec3(*(self.cam_pos + self.right_cam_offset)),
                                     gymapi.Vec3(*(self.cam_pos + self.right_cam_offset + self.cam_lookat_offset)))

    def step(self, head_rmat, left_pose, right_pose, left_qpos, right_qpos):

        if self.print_freq:
            start = time.time()

        self.left_root_states[0:7] = torch.tensor(left_pose, dtype=float)
        self.right_root_states[0:7] = torch.tensor(right_pose, dtype=float)
        self.gym.set_actor_root_state_tensor(self.sim, gymtorch.unwrap_tensor(self.root_states))

        left_states = np.zeros(self.dof, dtype=gymapi.DofState.dtype)
        left_states['pos'] = left_qpos
        self.gym.set_actor_dof_states(self.env, self.left_handle, left_states, gymapi.STATE_POS)

        right_states = np.zeros(self.dof, dtype=gymapi.DofState.dtype)
        right_states['pos'] = right_qpos
        self.gym.set_actor_dof_states(self.env, self.right_handle, right_states, gymapi.STATE_POS)

        # step the physics
        self.gym.simulate(self.sim)
        self.gym.fetch_results(self.sim, True)
        self.gym.step_graphics(self.sim)
        self.gym.render_all_camera_sensors(self.sim)
        self.gym.refresh_actor_root_state_tensor(self.sim)


        curr_lookat_offset = self.cam_lookat_offset @ head_rmat.T
        curr_left_offset = self.left_cam_offset @ head_rmat.T
        curr_right_offset = self.right_cam_offset @ head_rmat.T

        self.gym.set_camera_location(self.left_camera_handle,
                                     self.env,
                                     gymapi.Vec3(*(self.cam_pos + curr_left_offset)),
                                     gymapi.Vec3(*(self.cam_pos + curr_left_offset + curr_lookat_offset)))
        self.gym.set_camera_location(self.right_camera_handle,
                                     self.env,
                                     gymapi.Vec3(*(self.cam_pos + curr_right_offset)),
                                     gymapi.Vec3(*(self.cam_pos + curr_right_offset + curr_lookat_offset)))
        left_image = self.gym.get_camera_image(self.sim, self.env, self.left_camera_handle, gymapi.IMAGE_COLOR)
        right_image = self.gym.get_camera_image(self.sim, self.env, self.right_camera_handle, gymapi.IMAGE_COLOR)
        left_image = left_image.reshape(left_image.shape[0], -1, 4)[..., :3]
        right_image = right_image.reshape(right_image.shape[0], -1, 4)[..., :3]

        self.gym.draw_viewer(self.viewer, self.sim, True)
        self.gym.sync_frame_time(self.sim)

        if self.print_freq:
            end = time.time()
            print('Frequency:', 1 / (end - start))

        return left_image, right_image

    def end(self):
        self.gym.destroy_viewer(self.viewer)
        self.gym.destroy_sim(self.sim)


if __name__ == '__main__':
    teleoperator = VuerTeleop('inspire_hand.yml')
    simulator = Sim()
    # server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    #
    # server_socket.bind(("0.0.0.0", 12345))
    # server_socket.listen(1)
    # print("[*] 等待客户端连接...")
    # # server_socket.setblocking(False)  # 设置非阻塞模式
    # client_socket = None
    # client_socket, addr = server_socket.accept()
    try:
        while True:
            head_rmat, left_pose, right_pose, left_qpos, right_qpos = teleoperator.step()
            # 设置为单位阵
            # head_rmat= np.identity(3)
            # left_position = np.array([-0.6, 0.0, 1.6])
            # right_position = np.array([0.6, 0.0, 1.6])
            # left_orientation = np.array([0.0, 0.0, 0.0, 1.0])  # 单位四元数，无旋转
            #
            # left_pose = np.concatenate([left_position, left_orientation])
            # right_pose=np.concatenate([right_position, left_orientation])
            # left_qpos = np.array([0,0,0,1.47,1.47,0.0,0,0,0,0,0,0])
            # # 食指，食指中间，中指，中指中间，小指，0，无名指，0，大拇指旋转，0，大拇指弯曲，0）
            # right_qpos = np.array([0,0,0,0,1.47,0.0,0,0,0,0,1.47,0])
            left_img, right_img = simulator.step(head_rmat, left_pose, right_pose, left_qpos, right_qpos)
            np.copyto(teleoperator.img_array, np.hstack((left_img, right_img)))
            # print("right_qpos", right_qpos)
            # right_qpos=np.array([1.47,0,0,0,1.47,0.0])
            # try:
            #     if client_socket is not None:
            #
            #         com_right_qpos = right_qpos[:6].tolist()
            #         data_str = ','.join(map("{:.4f}".format, com_right_qpos)) + '\n'
            #         client_socket.sendall(data_str.encode("utf-8"))
            #         print(f"发送数据: {data_str.strip()}")
            #     else:
            #         pass
            # except BrokenPipeError:
            #     pass



    except KeyboardInterrupt:
        simulator.end()
        teleoperator.shm.close()
        teleoperator.shm.unlink()
        exit(0)
