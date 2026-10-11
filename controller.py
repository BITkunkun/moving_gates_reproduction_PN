"""比例导航（PN）控制器，对应论文式 (4)-(15)。

只使用视觉估计的视线方向及变化率，不读取无人机/门的位置、速度、距离或尺寸。
向量使用世界坐标系；内部闭式解采用单位质量推力，输出时换算为牛顿。
"""

from dataclasses import dataclass

import numpy as np

from geometry import matrix_quat, pitch_roll, pn_frame


@dataclass(frozen=True)
class Guidance:
    """一帧视觉测量；世界系方向已由拍摄时刻的 IMU 姿态补偿。"""

    los_world: np.ndarray  # 从无人机指向门中心的单位视线向量
    los_rate: float  # 视线角速度的大小，单位 rad/s
    normal_world: np.ndarray  # PN 加速度的单位方向；首次测量时为零向量
    capture_time: float  # 拍摄时间，单位 s；不同于延迟后的接收时间


@dataclass(frozen=True)
class Command:
    """发送给动力学内环的指令，以及用于图表检查约束的诊断量。"""

    thrust_newton: float  # 四旋翼总推力，单位 N
    quaternion: np.ndarray  # 机体系到世界系的目标姿态，顺序 [w, x, y, z]
    normal_acceleration: np.ndarray  # 满足约束后的世界系法向加速度，单位 m/s²
    requested_normal: np.ndarray  # 未限幅的 PN 法向加速度，单位 m/s²
    saturation_scale: float  # 法向加速度保留比例，范围 [0, 1]
    commanded_gamma_deg: float  # 指令相机轴与所测视线的几何夹角，单位度
    constraint_gamma_deg: float  # 论文式 (15) 使用的保守代理夹角，单位度


class PNController:
    """把 PN 法向加速度转换为满足推力上限及视野约束的姿态/推力。"""

    def __init__(self, mass=2.0, thrust_max=39.24, k_pn=2.10,
                 speed_bound=15.0, gamma_max_deg=21.05):
        if mass <= 0 or thrust_max < mass*9.81 or speed_bound <= 0 or k_pn < 0:
            raise ValueError("Invalid mass, thrust, navigation gain or speed bound")
        if not 0 < gamma_max_deg < 90:
            raise ValueError("gamma_max_deg must lie in (0, 90)")
        self.mass, self.thrust_max = mass, thrust_max
        self.k_pn, self.speed_bound = k_pn, speed_bound
        self.gamma_max = np.deg2rad(gamma_max_deg)

    def command(self, measurement: Guidance) -> Command:
        """给定视觉测量，按论文闭式解计算一条控制指令。"""
        # frame 的列是 PN 基向量在世界系中的表示；转置用于世界系 -> PN 系。
        frame = pn_frame(measurement.los_world)
        # 式 (5)：固定速度上界代替未知的相对速度，避免引入速度状态估计。
        requested = self.k_pn*self.speed_bound*measurement.los_rate*measurement.normal_world
        local_normal = frame.T @ requested
        gravity = frame.T @ np.array([0.0, 0.0, -9.81])
        cap = self.thrust_max / self.mass  # 单位质量推力上限，单位 m/s²
        cosine = np.cos(self.gamma_max)

        def feasible(scale):
            """检查缩放后的法向加速度是否存在满足式 (11)、(15) 的推力。"""
            lateral = scale*local_normal[1]
            # 推力的竖直分量还需抵消 PN 系中的重力分量。
            vertical = scale*local_normal[2] - gravity[2]
            upper = min(cap, vertical/cosine)
            # 所需推力至少为法向两分量的合成值，且不能超过物理/视野上限。
            return vertical > 0 and np.hypot(lateral, vertical) <= upper + 1e-10

        scale = 1.0
        if not feasible(scale):
            # 法向请求过大时，式 (9) 无可行解。二分查找最大的可行缩放比例，
            # 保持方向、降低幅值，避免反三角函数收到超出定义域的输入。
            lo, hi = 0.0, 1.0
            for _ in range(28):
                mid = (lo+hi)/2
                if feasible(mid):
                    lo = mid
                else:
                    hi = mid
            scale = lo

        lateral = scale*local_normal[1]
        vertical = scale*local_normal[2] - gravity[2]
        # 式 (13) 的前向推力随总推力单调增加，故取式 (15) 允许的最大值。
        thrust = min(cap, vertical/cosine)
        # 式 (12)：先求滚转，再由剩余推力分量求俯仰；clip 消除浮点越界。
        roll = np.arcsin(np.clip(-lateral/thrust, -1.0, 1.0))
        denominator = np.sqrt(max(thrust**2-lateral**2, 1e-16))
        pitch = np.arccos(np.clip(vertical/denominator, -1.0, 1.0))
        orientation = frame @ pitch_roll(pitch, roll)
        # 几何夹角与论文的代理夹角不同，分别记录，便于核查保守视野约束。
        actual_gamma = np.arccos(np.clip(measurement.los_world @ orientation[:, 0], -1, 1))
        proxy_gamma = np.arccos(np.clip(vertical/thrust, -1, 1))
        return Command(
            self.mass*thrust, matrix_quat(orientation), scale*requested,
            requested, scale, np.rad2deg(actual_gamma), np.rad2deg(proxy_gamma),
        )
