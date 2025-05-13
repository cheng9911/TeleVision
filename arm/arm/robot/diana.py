from typing import List

import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
import modern_robotics as mr
from scipy.spatial.transform import Rotation as R
from tracikpy import TracIKSolver
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from arm.robot.robot_config import RobotConfig
from arm.robot.robot import Robot, get_transformation_mdh, wrap
from arm.geometry import Geometry3D, Capsule
from arm.utils import MathUtils

class Diana(Robot):

    def __init__(self) -> None:
        super().__init__()
        self.ik_solver = TracIKSolver(
                "/home/sun/Documents/GitHub/imitation_learning_idp3/tracikpy/data/diana_v2.urdf",
                "base",
                "link_7",solve_type="Distance",timeout=0.01,epsilon=1e-5)
        self._dof = 7
        self.q0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ,0.0]
        self.q_current=[0.0, 0.564, 0, 1.84, 0.089, -0.504,0]
    def ikine(self, Tep: SE3) -> np.ndarray:
        """
        改进的逆运动学方法
        :param Tep: 目标位姿
        :return: 7维关节角数组
        """
        # print("q_current", self.q_current)
        initial_q = self.q_current

        T_adjust = SE3.Rx(np.pi) * SE3.Rz(np.pi / 2)
        # print("actual pose", self._base.inv() * Tep * self._tool.inv()*T_adjust.inv())
        q_sol = self.ik_solver.ik(
            self._base.inv() * Tep * self._tool.inv()*T_adjust.inv(),
            initial_q,  # 使用当前关节角作为初始值

        )


        if q_sol is not None:
            # self.q_current=np.array(q_sol)

            return q_sol
        else:
            print("逆运动学求解失败")
        return np.array([])

    def fkine(self, q: List[float]) -> SE3:

        # return self.fkine(q)
        """改进后的正向运动学方法"""
        T_base = self.ik_solver.fk(q)  # 直接接收返回值
        T = np.dot(self._base, T_base)

        if T is None:
            raise ValueError("正向运动学求解失败")

        return SE3(T)  # 转换为SE3对象
    def fkine_tool(self, q: List[float]) -> SE3:

        # return self.fkine(q)
        """改进后的正向运动学方法"""
        T_base = self.ik_solver.fk(q)  # 直接接收返回值
        T=np.dot(self._base,T_base)
        T_adjust = SE3.Rx(np.pi) * SE3.Rz(np.pi / 2)
        T=T*T_adjust*self._tool

        # quat = np.array([-0.4480736, 0.8939967, 0, 0])
        #
        # # 位置 pos = (0, 0, 0)
        # pos = np.array([0, 0, 0])
        #
        # # 将四元数转换为旋转矩阵 (3x3)
        # rotation_matrix = R.from_quat(quat[[1, 2, 3, 0]]).as_matrix()  # 需要调整顺序 (qx, qy, qz, qw)
        # T_flange = np.eye(4)
        # T_flange[:3, :3] = rotation_matrix
        # T_flange[:3, 3] = pos
        # T=np.dot(T,T_flange)
        # print("hello",self._base)
        # print("hello", T)
        if T is None:
            raise ValueError("正向运动学求解失败")

        return SE3(T)  # 转换为SE3对象
    def move_cartesian(self, T: SE3):
        q = self.ikine(T)

        if q.size != 0:
            self.q0 = q[:]

    def move_cartesian_with_avoidance(self, T: SE3):
        q = self.ikine_with_avoidance(T)
        if q.size != 0:
            self.q0 = q[:]


if __name__ == '__main__':
    ur_robot = Diana()
    q0 = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6,0.7]
    T1 = ur_robot.fkine(q0)
    print(T1)
    # ur_robot.move_cartesian(T1)
    # q_new = ur_robot.get_joint()
    # print(q_new)
   # 给定SE3矩阵，求解逆运动学

    ik_solver = TracIKSolver(
                "/home/sun/Documents/GitHub/imitation_learning_idp3/tracikpy/data/diana_v2.urdf",
                "base",
                "link_7",solve_type="Distance",timeout=0.005,epsilon=1e-4)
    dof = 7
    q0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ,0.0]
    q_current=np.array([0.0, 0.471, 0, 2.01, 0.089, -0.628,0])
    matrix = np.array([
        [0.9703, -0.06578, -0.2329, 0.6923],
        [0.07783, 0.996, 0.04292, 0.01754],
        [0.2291, -0.05977, 0.9716, 0.2351],
        [0.0, 0.0, 0.0, 1.0]
    ], dtype=float)

    # 提取旋转矩阵 R 和平移向量 t

    i=1
    # while i<100:
    #
    #     q_sol=ik_solver.ik(matrix ,q_current )
    #     print(q_sol)
    #     i=i+1
    T_adjust = SE3.Rx(np.pi) * SE3.Rz(np.pi / 2)
    print("actual pose", T_adjust)
    # T4 [1.44632959 0.73519781 0.89      ]
    # states[1.44430055
    # 0.72089982
    # 0.88026866]
    # actions[1.44632827
    # 0.73519891
    # 0.89000089]
