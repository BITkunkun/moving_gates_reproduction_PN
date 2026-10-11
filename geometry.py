"""旋转与坐标变换工具。

四元数统一使用 [w, x, y, z]；机体系采用 FLU：x 向前、y 向左、z 向上。
姿态矩阵把机体系列向量转换到世界系，逆变换使用矩阵转置。
"""

import numpy as np
from scipy.spatial.transform import Rotation


def unit(vector):
    """归一化非零向量；零向量不能定义方向。"""
    vector = np.asarray(vector, dtype=float)
    norm = np.linalg.norm(vector)
    if norm < 1e-12:
        raise ValueError("Cannot normalize a zero vector")
    return vector / norm


def pn_frame(los):
    """构造论文式 (6) 的正交 PN 基，矩阵的列在世界系中表示。

    x 轴沿视线，y 轴垂直于世界竖直方向和视线，z 轴由右手法则确定。
    视线完全竖直时叉积退化，不满足本文 PN 坐标系的适用条件。
    """
    x = unit(los)
    y = np.cross([0.0, 0.0, 1.0], x)
    if np.linalg.norm(y) < 1e-8:
        raise ValueError("Vertical LOS is outside the paper's PN-frame assumptions")
    y = unit(y)
    return np.column_stack((x, y, np.cross(x, y)))


def quat_multiply(a, b):
    """Hamilton 四元数乘法，用于姿态复合、误差与增量更新。"""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ])


def quat_matrix(q):
    """把单位四元数转换为机体系 -> 世界系的旋转矩阵。"""
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def matrix_quat(matrix):
    """将旋转矩阵转换为本项目的 [w, x, y, z] 顺序。"""
    # SciPy 输出 [x, y, z, w]，需要重排，不能直接交给动力学使用。
    xyzw = Rotation.from_matrix(matrix).as_quat()
    return xyzw[[3, 0, 1, 2]]


def rotation_increment(rotation_vector):
    """由轴角向量（rad）生成增量四元数；小角度用一阶近似避免除零。"""
    angle = np.linalg.norm(rotation_vector)
    if angle < 1e-10:
        return unit(np.r_[1.0, 0.5*np.asarray(rotation_vector)])
    return np.r_[np.cos(angle/2), np.sin(angle/2)*rotation_vector/angle]


def pitch_roll(pitch, roll):
    """按论文式 (10) 返回 Ry(pitch) Rx(roll)，角度单位为 rad。"""
    cp, sp, cr, sr = np.cos(pitch), np.sin(pitch), np.cos(roll), np.sin(roll)
    return np.array([[cp, sp*sr, sp*cr], [0.0, cr, -sr], [-sp, cp*sr, cp*cr]])


def angle_between(a, b):
    """计算两个单位方向的夹角（rad）；atan2 在小夹角时更稳定。"""
    return np.arctan2(np.linalg.norm(np.cross(a, b)), np.clip(np.dot(a, b), -1, 1))
