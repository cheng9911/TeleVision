import numpy as np
from tracikpy import TracIKSolver
from scipy.spatial.transform import Rotation


def matrix_to_quaternion(rot_matrix: np.ndarray) -> np.ndarray:
    """
    将3x3旋转矩阵转换为四元数 [x, y, z, w]
    """
    if rot_matrix.shape != (3, 3):
        raise ValueError("输入必须是3x3的旋转矩阵")

    r = Rotation.from_matrix(rot_matrix)
    return r.as_quat()  # 返回格式为[x, y, z, w]
ee_pose = np.array([[ 0.0525767 , -0.64690764, -0.7607537 , 0.        ],
                    [-0.90099786, -0.35923817,  0.24320937, 0.2       ],
                    [-0.43062577,  0.67265031, -0.60174996, 0.4       ],
                    [ 0.        ,  0.        ,  0.        , 1.        ]])

ik_solver = TracIKSolver(
    "/home/sun/Documents/GitHub/imitation_learning_idp3/tracikpy/data/diana_v2.urdf",
    "base",
    "link_7",

)
qout = ik_solver.ik(ee_pose, qinit=np.zeros(ik_solver.number_of_joints))
print(qout)
from tracikpy import TracIKSolver

# 初始化 TracIK 求解器
ik_solver = TracIKSolver(
    "/home/sun/Documents/Diana7_ros2/src/diana7_description/urdf/diana_v2.urdf",
    "base",
    "link_7",
)

# 设置关节角度
joint_angles = [0.0, 0.564, 0, 1.84, 0.089, -0.504,0]

# 计算正运动学
pose = ik_solver.fk(joint_angles)
print("End-effector pose:", pose)
# pose的旋转改为四元数
rot_matrix = pose[:3, :3]
quat = matrix_to_quaternion(rot_matrix)
print("End-effector pose quaternion:", quat)
