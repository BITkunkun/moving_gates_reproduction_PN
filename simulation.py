"""三维四旋翼、单目相机及只做平移的圆形门仿真。

真实状态仅用于动力学、合成相机和评分；控制器只接收由像素计算的 Guidance。
世界系 z 轴向上，长度单位为 m，时间单位为 s，角速度单位为 rad/s，推力单位为 N。
"""

from collections import deque
from dataclasses import asdict, dataclass
import json
from pathlib import Path

import numpy as np
from scipy.optimize import brentq

from controller import Command, Guidance, PNController
from geometry import (angle_between, matrix_quat, pn_frame, quat_matrix,
                      quat_multiply, rotation_increment, unit)


MOTIONS = ("stationary", "linear_slow", "linear_fast", "planar", "knot")
# 两种论文奖励优化出的参数，元组顺序为 (PN 增益, 视野上限/度)。
PRESETS = {"distance_time": (2.10, 21.05), "distance": (1.11, 8.91)}


@dataclass(frozen=True)
class Config:
    """仿真与控制参数；论文来源及未公开参数的假设见 README 参数表。"""

    # 场景选择与 PN 外环参数。
    motion: str = "knot"
    seed: int = 7
    preset: str = "distance_time"
    k_pn: float = 2.10
    gamma_max_deg: float = 21.05
    speed_bound: float = 15.0  # 固定相对速度上界，单位 m/s；不从真值更新
    mass: float = 2.0  # 单位 kg
    thrust_max: float = 39.24  # 单位 N
    initial_distance: float = 0.0  # 0 表示在 [20, 30] m 内随机采样门轨道中心距离
    initial_speed: float = 10.0
    gate_radius: float = 1.0
    # 视觉测量及环境干扰参数。
    delay: float = 0.0  # 拍摄到接收的延迟，单位 s
    pixel_noise: float = 0.4  # 检测框四个坐标的噪声标准差，单位 pixel
    filter_tau: float = 0.06  # LOS 低通滤波时间常数，单位 s；0 表示不滤波
    wind_y: float = 0.0  # 世界系 y 方向风速，单位 m/s
    # 动力学与内环参数；dt 必须整除视觉/外环控制周期。
    dt: float = 1/240  # 动力学积分步长，单位 s
    control_hz: float = 30.0  # 视觉采样与制导频率，单位 Hz
    duration: float = 12.0
    attitude_tau: float = 0.16
    rate_tau: float = 0.04
    motor_tau: float = 0.03
    drag_linear: float = 0.08  # 线性阻力加速度系数，单位 s⁻¹
    drag_quadratic: float = 0.012  # 二次阻力加速度系数，单位 m⁻¹
    camera_half_fov_deg: float = 33.5  # 视线可见锥的半角，单位度

    def __post_init__(self):
        if self.motion not in MOTIONS:
            raise ValueError(f"Unknown motion: {self.motion}")
        if self.preset not in PRESETS:
            raise ValueError(f"Unknown preset: {self.preset}")
        if not 0 < self.gamma_max_deg <= self.camera_half_fov_deg < 90:
            raise ValueError("Require 0 < gamma_max <= camera half-FOV < 90")
        if min(self.dt, self.control_hz, self.duration, self.gate_radius,
               self.attitude_tau, self.rate_tau, self.motor_tau, self.mass) <= 0:
            raise ValueError("Time steps, rates, mass and gate radius must be positive")
        if min(self.delay, self.pixel_noise, self.filter_tau, self.initial_distance,
               self.initial_speed, self.drag_linear, self.drag_quadratic) < 0:
            raise ValueError("Delay, noise, distance, speed and drag must be nonnegative")
        ratio = 1/(self.dt*self.control_hz)
        if abs(ratio-round(ratio)) > 1e-8:
            raise ValueError("Control period must be an integer multiple of dt")


class GateMotion:
    """论文 V-A 的五种门运动；平面/knot 运动带随机初相位。"""
    def __init__(self, name, anchor, phase=0.0):
        self.name, self.anchor, self.phase = name, np.asarray(anchor, dtype=float), phase

    def position(self, t):
        """返回时间 t（s）对应的世界系门中心（m），支持单时刻或时间数组。"""
        t = np.asarray(t)
        zero = np.zeros_like(t)
        if self.name == "stationary":
            offset = np.stack((zero, zero, zero), axis=-1)
        elif self.name.startswith("linear"):
            speed = 2.5 if self.name == "linear_slow" else 5.0
            offset = np.stack((zero, speed*t, zero), axis=-1)
        elif self.name == "planar":
            # 论文平面轨迹：尺度 2 m；角频率 0.7 rad/s 是本仿真的参数假设。
            a = 0.7*t + self.phase
            offset = 2.0*np.stack((np.cos(a), 0.5*np.cos(2*a), zero), axis=-1)
        else:
            # 论文三维 knot 轨迹：尺度 1 m；角频率假设为 0.2 rad/s。
            a = 0.2*t + self.phase
            offset = np.stack((np.sin(3*a)*(1+0.5*np.cos(2*a)),
                               np.cos(3*a)*np.sin(2*a), np.sin(4*a)), axis=-1)
        return self.anchor + offset


class Camera:
    """针孔相机：光轴沿机体系 +x，像素 u 向右、v 向下。"""
    width, height = 640, 480

    def __init__(self, half_fov_deg):
        self.half_fov = np.deg2rad(half_fov_deg)
        # 可见圆锥内接于图像短边，保证论文的角度约束落在图像范围内。
        self.focal = (self.height/2)/np.tan(self.half_fov)
        self.cx, self.cy = self.width/2, self.height/2

    def project(self, points_world, position, rotation):
        """将世界系门边界投影为像素；相机后方的点返回 NaN。"""
        # points_world 的每行是一个点，故行向量右乘 rotation 等价于逆旋转。
        body = (np.asarray(points_world)-position) @ rotation
        depth = body[..., 0]
        safe = np.where(depth > 1e-6, depth, np.nan)
        return np.stack((self.cx-self.focal*body[..., 1]/safe,
                         self.cy-self.focal*body[..., 2]/safe), axis=-1)

    def pixel_to_body(self, u, v):
        """从检测框中心恢复机体系单位视线；负号来自 FLU 与像素轴的反向。"""
        return unit([1.0, -(u-self.cx)/self.focal, -(v-self.cy)/self.focal])


@dataclass(frozen=True)
class Detection:
    """一帧制导测量与检测框；bbox 顺序为 [u_min, v_min, u_max, v_max]。"""

    guidance: Guidance
    bbox: np.ndarray


class Vision:
    """视觉链路：投影检测框 -> LOS 滤波 -> 世界系角速度 -> 延迟队列。"""
    def __init__(self, config, camera, rng):
        self.config, self.camera, self.rng = config, camera, rng
        self.previous = None
        self.previous_time = None
        self.queue = deque()
        self.latest = None
        self.attempts = self.detected = 0

    def capture(self, t, center, ring, position, rotation):
        """合成拍摄时刻 t 的检测；这里使用真值模拟传感器，不传给控制器。"""
        self.attempts += 1
        # 检测失败也消耗一次噪声采样，使同一种子下的不同控制参数便于对照。
        noise = self.rng.normal(0, self.config.pixel_noise, 4)
        relative = center-position
        true_los = unit(relative)
        if angle_between(true_los, rotation[:, 0]) > self.camera.half_fov:
            return
        pixels = self.camera.project(ring, position, rotation)
        if not np.all(np.isfinite(pixels)):
            return
        inside = ((pixels[:, 0] >= 0) & (pixels[:, 0] <= self.camera.width)
                  & (pixels[:, 1] >= 0) & (pixels[:, 1] <= self.camera.height))
        if not np.any(inside):
            return  # 很近时门边缘可能全部离开图像，模拟检测丢失。
        low, high = pixels.min(axis=0), pixels.max(axis=0)
        bbox = np.r_[low, high] + noise
        u, v = (bbox[:2]+bbox[2:])/2
        # 必须用拍摄时刻的 IMU 姿态补偿相机自转，避免把机体转动误认为门运动。
        los = rotation @ self.camera.pixel_to_body(u, v)
        if self.previous is None:
            # 首帧没有历史方向，无法求导，先令视线角速度和法向加速度为零。
            rate, normal = 0.0, np.zeros(3)
        else:
            elapsed = t-self.previous_time
            # 按实际有效测量间隔计算滤波权重；丢帧后仍使用真实 elapsed。
            alpha = 1.0 if self.config.filter_tau == 0 else -np.expm1(-elapsed/self.config.filter_tau)
            los = unit((1-alpha)*self.previous + alpha*los)
            axis = np.cross(los, self.previous)  # 论文 k_l = l × l_prev，顺序决定方向。
            rate = angle_between(los, self.previous)/elapsed
            normal = unit(np.cross(los, axis)) if np.linalg.norm(axis) > 1e-10 else np.zeros(3)
        self.previous, self.previous_time = los, t
        sample = Detection(Guidance(los, rate, normal, t), bbox)
        # 样本保持拍摄时间，队列另存可接收时间；不能用当前姿态修正旧图像。
        self.queue.append((t+self.config.delay, sample))
        self.detected += 1

    def deliver(self, t):
        """交付截至时间 t 已到达的最新检测；无新帧时保持上次有效测量。"""
        while self.queue and self.queue[0][0] <= t+1e-9:
            _, self.latest = self.queue.popleft()
        return self.latest


class Quadrotor:
    """论文式 (3)：平移用二阶积分，姿态用旋转增量四元数更新。"""
    def __init__(self, config, position, velocity, quaternion):
        self.config = config
        # p/v 为世界系位置/速度，q 为机体 -> 世界姿态，omega 为机体系角速度。
        self.p, self.v, self.q = position.copy(), velocity.copy(), quaternion.copy()
        self.omega = np.zeros(3)
        self.thrust = config.mass*9.81

    def acceleration(self, velocity, rotation, thrust):
        """由推力、重力和相对风速阻力计算世界系加速度，单位 m/s²。"""
        # 阻力依据相对空气速度，在机体系计算后再转换回世界系。
        relative_body = rotation.T @ (velocity-np.array([0.0, self.config.wind_y, 0.0]))
        drag_body = (self.config.drag_linear*relative_body
                     + self.config.drag_quadratic*np.linalg.norm(relative_body)*relative_body)
        return rotation[:, 2]*thrust/self.config.mass - rotation@drag_body + [0.0, 0.0, -9.81]

    def step(self, command, dt):
        """将内环和动力学推进 dt 秒；整个步长内保持外环指令不变。"""
        c = self.config
        conjugate = self.q*np.array([1.0, -1.0, -1.0, -1.0])
        error = quat_multiply(conjugate, command.quaternion)
        # q 与 -q 表示同一姿态；按误差实部选符号，沿较短的旋转方向跟踪。
        omega_c = (2/c.attitude_tau)*(1 if error[0] >= 0 else -1)*error[1:]
        omega_next = omega_c+(self.omega-omega_c)*np.exp(-dt/c.rate_tau)
        # 一阶角速度/电机环使用指数响应；平均角速度构造本步姿态增量。
        q_next = unit(quat_multiply(self.q, rotation_increment(0.5*(self.omega+omega_next)*dt)))
        thrust_next = command.thrust_newton+(self.thrust-command.thrust_newton)*np.exp(-dt/c.motor_tau)
        acceleration = self.acceleration(self.v, quat_matrix(self.q), self.thrust)
        # 先预测末速度，再平均首末加速度校正速度；位置按平均速度积分。
        predicted_v = self.v+dt*acceleration
        acceleration_next = self.acceleration(predicted_v, quat_matrix(q_next), thrust_next)
        v_next = self.v+0.5*dt*(acceleration+acceleration_next)
        self.p += 0.5*dt*(self.v+v_next)
        self.v, self.q, self.omega, self.thrust = v_next, q_next, omega_next, thrust_next


@dataclass
class Result:
    """一次仿真的时间序列、评分及门几何，供可视化和实验统计使用。"""

    config: Config
    arrays: dict
    metrics: dict
    gate: GateMotion
    gate_frame: np.ndarray

    def save(self, directory):
        """NPZ 保存完整数组，CSV 保存常用轨迹列，JSON 保存配置和性能指标。"""
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(directory/"trajectory.npz", **self.arrays)
        (directory/"metrics.json").write_text(json.dumps(
            {"config": asdict(self.config), "metrics": self.metrics}, indent=2, allow_nan=False), encoding="utf-8")
        columns = np.column_stack((self.arrays["time"], self.arrays["position"],
                                   self.arrays["velocity"], self.arrays["gate"],
                                   self.arrays["gamma_deg"], self.arrays["thrust"]))
        np.savetxt(directory/"trajectory.csv", columns, delimiter=",", comments="",
                   header="time,x,y,z,vx,vy,vz,gate_x,gate_y,gate_z,gamma_deg,thrust_N")


def simulate(config=Config()):
    """运行单门穿越：初始化 -> 视觉/制导采样 -> 动力学积分 -> 事件评分。"""
    # 固定种子保证场景可重现；视觉噪声使用独立随机数生成器。
    rng = np.random.default_rng(config.seed)
    distance = config.initial_distance or rng.uniform(20, 30)
    phase = rng.uniform(0, 2*np.pi)
    gate = GateMotion(config.motion, [distance, 0, 3], phase)
    position = np.array([0.0, 0.0, 3.0])
    initial_frame = pn_frame(gate.position(0)-position)
    gate_frame = initial_frame.copy()  # 门初始朝向无人机，此后仅平移、不旋转。
    angle, azimuth = np.deg2rad(rng.uniform(-10, 10)), rng.uniform(0, 2*np.pi)
    velocity = config.initial_speed*(initial_frame @ np.array([
        np.cos(angle), np.sin(angle)*np.cos(azimuth), np.sin(angle)*np.sin(azimuth)]))
    plant = Quadrotor(config, position, velocity, matrix_quat(initial_frame))
    controller = PNController(config.mass, config.thrust_max, config.k_pn,
                              config.speed_bound, config.gamma_max_deg)
    camera = Camera(config.camera_half_fov_deg)
    vision = Vision(config, camera, np.random.default_rng(config.seed+1_000_000))
    # 用固定门平面的两个切向基构造圆环；圆环用于成像和最终平面内评分。
    circle_angle = np.linspace(0, 2*np.pi, 96, endpoint=False)
    ring_offset = config.gate_radius*(np.cos(circle_angle)[:, None]*gate_frame[:, 1]
                                     + np.sin(circle_angle)[:, None]*gate_frame[:, 2])
    command = Command(config.mass*9.81, plant.q.copy(), np.zeros(3), np.zeros(3), 1.0, 0.0, 0.0)
    records = []
    crossed, crossing_time, miss_distance = False, None, None
    status, peak_speed = "timeout", np.linalg.norm(plant.v)
    stride = round(1/(config.dt*config.control_hz))
    max_steps = int(round(config.duration/config.dt))
    saturation_count = command_count = 0

    def record(t):
        """记录当前状态和指令；真值只供评分/绘图，不作为外环输入。"""
        center = gate.position(t)
        rotation = quat_matrix(plant.q)
        detection = vision.latest
        records.append({
            "time": t, "position": plant.p.copy(), "velocity": plant.v.copy(),
            "quaternion": plant.q.copy(), "gate": center,
            "los": detection.guidance.los_world if detection else np.full(3, np.nan),
            "los_rate": detection.guidance.los_rate if detection else 0.0,
            "gamma_deg": np.rad2deg(angle_between(unit(center-plant.p), rotation[:, 0])),
            "command_gamma_deg": command.commanded_gamma_deg,
            "constraint_gamma_deg": command.constraint_gamma_deg,
            "thrust": plant.thrust, "saturation_scale": command.saturation_scale,
            "normal_acceleration": command.normal_acceleration,
            "requested_normal": command.requested_normal,
            "measurement_age": t-detection.guidance.capture_time if detection else np.nan,
            "bbox": detection.bbox if detection else np.full(4, np.nan),
        })

    for step in range(max_steps):
        t = step*config.dt
        if step % stride == 0:
            # 外环只在 30 Hz 采样时更新；其他积分步保持上一条控制指令。
            center = gate.position(t)
            vision.capture(t, center, center+ring_offset, plant.p, quat_matrix(plant.q))
            detection = vision.deliver(t)
            if detection is not None:
                try:
                    command = controller.command(detection.guidance)
                except ValueError:
                    status = "invalid_pn_frame"
                    record(t)
                    break
                command_count += 1
                saturation_count += command.saturation_scale < 0.999
            record(t)

        # 符号距离沿固定门法向计算；门平面本身也在随时间平移。
        previous_position = plant.p.copy()
        previous_q, previous_v, previous_thrust = plant.q.copy(), plant.v.copy(), plant.thrust
        signed_before = (gate.position(t)-previous_position) @ gate_frame[:, 0]
        plant.step(command, config.dt)
        next_t = t+config.dt
        peak_speed = max(peak_speed, np.linalg.norm(plant.v))
        signed_after = (gate.position(next_t)-plant.p) @ gate_frame[:, 0]
        if signed_before > 0 >= signed_after:
            # 首末符号改变表示穿过移动门平面。步内求根得到穿越时刻，
            # 而不是把最近距离或世界 x 坐标相等直接视为穿门。
            def signed_plane(fraction):
                p = previous_position+fraction*(plant.p-previous_position)
                return (gate.position(t+fraction*config.dt)-p) @ gate_frame[:, 0]
            fraction = brentq(signed_plane, 0.0, 1.0)
            crossing_time = t+fraction*config.dt
            crossing_position = previous_position+fraction*(plant.p-previous_position)
            relative = crossing_position-gate.position(crossing_time)
            # 去掉法向分量后计算门平面内误差，按点质心到圆心距离判定成功。
            miss_distance = float(np.linalg.norm(relative-(relative@gate_frame[:, 0])*gate_frame[:, 0]))
            crossed = True
            status = "passed" if miss_distance <= config.gate_radius else "missed"
            # 将末帧插值到穿越瞬间，保证轨迹终点与评分使用同一时刻。
            plant.p = crossing_position
            plant.v = (1-fraction)*previous_v+fraction*plant.v
            plant.q = unit((1-fraction)*previous_q+fraction*plant.q)
            plant.thrust = (1-fraction)*previous_thrust+fraction*plant.thrust
            record(crossing_time)
            break
        if plant.p[2] < 0.1:
            status = "ground_contact"
            record(next_t)
            break
        if step == max_steps-1:
            record(next_t)

    arrays = {key: np.asarray([row[key] for row in records]) for key in records[0]}
    # 分别统计真实相机角度、指令角度和法向饱和比例，避免混淆不同约束。
    metrics = {
        "motion": config.motion, "seed": config.seed, "preset": config.preset,
        "status": status, "crossed": crossed, "success": status == "passed",
        "crossing_time_s": crossing_time, "miss_distance_m": miss_distance,
        "peak_speed_m_s": float(peak_speed),
        "maximum_actual_gamma_deg": float(np.max(arrays["gamma_deg"])),
        "maximum_command_gamma_deg": float(np.max(arrays["command_gamma_deg"])),
        "fov_sample_fraction": float(np.mean(arrays["gamma_deg"] <= config.camera_half_fov_deg)),
        "normal_saturation_fraction": saturation_count/max(command_count, 1),
        "detection_fraction": vision.detected/max(vision.attempts, 1),
        "initial_distance_m": float(np.linalg.norm(gate.position(0)-position)),
    }
    return Result(config, arrays, metrics, gate, gate_frame)
