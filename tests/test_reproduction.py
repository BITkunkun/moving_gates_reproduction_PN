"""回归验证：坐标变换、视觉延迟、控制约束、动力学与完整穿门流程。"""

from dataclasses import replace
import unittest

import numpy as np

from controller import Command, Guidance, PNController
from experiments import gp_ucb, reward
from geometry import matrix_quat, pitch_roll, pn_frame, quat_matrix, unit
from simulation import Camera, Config, MOTIONS, Quadrotor, Vision, simulate


class GeometryAndCameraTests(unittest.TestCase):
    """检查几何约定，以及相机自转不会被误认为目标视线运动。"""

    def test_frame_is_orthonormal_and_quaternion_roundtrip(self):
        for los in ([1, 0, 0], [3, -2, 1], [1, 2, -3]):
            frame = pn_frame(los)
            np.testing.assert_allclose(frame.T@frame, np.eye(3), atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(frame), 1)
            np.testing.assert_allclose(quat_matrix(matrix_quat(frame)), frame, atol=1e-12)

    def test_pixels_recover_world_direction(self):
        camera = Camera(33.5)
        rotation = pitch_roll(0.15, -0.1)
        position, point = np.array([1, 2, 3]), np.array([21, 5, 4])
        pixels = camera.project(point, position, rotation)
        recovered = rotation@camera.pixel_to_body(*pixels)
        np.testing.assert_allclose(recovered, unit(point-position), atol=1e-12)

    def test_camera_rotation_does_not_create_target_motion(self):
        # 目标与无人机位置固定，仅旋转相机；IMU 补偿后的世界系 LOS 应不变。
        config = Config(pixel_noise=0, filter_tau=0)
        vision = Vision(config, Camera(33.5), np.random.default_rng(0))
        center, position = np.array([20.0, 0, 3]), np.array([0.0, 0, 3])
        ring = np.repeat(center[None, :], 10, axis=0)
        vision.capture(0, center, ring, position, np.eye(3))
        vision.capture(1/30, center, ring, position, pitch_roll(0.2, 0.1))
        sample = vision.deliver(1/30).guidance
        self.assertLess(sample.los_rate, 1e-10)
        np.testing.assert_allclose(sample.los_world, [1, 0, 0], atol=1e-12)

    def test_delay_delivers_capture_time_measurement(self):
        config = Config(delay=0.1, pixel_noise=0)
        vision = Vision(config, Camera(33.5), np.random.default_rng(0))
        center = np.array([20.0, 0, 3])
        vision.capture(0, center, np.repeat(center[None, :], 10, axis=0),
                       np.array([0.0, 0, 3]), np.eye(3))
        self.assertIsNone(vision.deliver(0.099))
        self.assertEqual(vision.deliver(0.1).guidance.capture_time, 0)


class ControllerTests(unittest.TestCase):
    """检查闭式解、推力/视野约束，以及不可行 PN 请求的限幅。"""

    def check_command(self, los, normal, rate):
        """把实际推力方向还原为加速度，独立检查法向分量和控制约束。"""
        controller = PNController()
        command = controller.command(Guidance(los, rate, normal, 0))
        frame, rotation = pn_frame(los), quat_matrix(command.quaternion)
        acceleration = frame.T@(rotation[:, 2]*command.thrust_newton/2+[0, 0, -9.81])
        expected = frame.T@command.normal_acceleration
        np.testing.assert_allclose(acceleration[1:], expected[1:], atol=1e-8)
        self.assertGreaterEqual(command.thrust_newton, 0)
        self.assertLessEqual(command.thrust_newton, 39.24+1e-9)
        self.assertLessEqual(command.commanded_gamma_deg, 21.05+1e-8)
        self.assertLessEqual(command.constraint_gamma_deg, 21.05+1e-8)
        self.assertLessEqual(command.commanded_gamma_deg, command.constraint_gamma_deg+1e-8)
        return command, acceleration

    def test_closed_form_matches_grid_optimum(self):
        # 用独立推力网格搜索核对闭式前向加速度，而不是重复同一个闭式公式。
        los = unit([1.0, 0.2, 0.1])
        frame = pn_frame(los)
        command, acceleration = self.check_command(los, frame[:, 1], 0.03)
        normal = frame.T@command.normal_acceleration
        gravity = frame.T@np.array([0, 0, -9.81])
        y, z = normal[1], normal[2]-gravity[2]
        thrusts = np.linspace(np.hypot(y, z), 39.24/2, 20000)
        feasible = thrusts[z/thrusts >= np.cos(np.deg2rad(21.05))]
        brute_force = np.sqrt(np.maximum(feasible**2-y*y-z*z, 0)).max()+gravity[0]
        self.assertGreaterEqual(acceleration[0], brute_force-1e-8)
        self.assertLess(acceleration[0]-brute_force, 0.005)

    def test_infeasible_normal_request_is_saturated(self):
        command, _ = self.check_command(np.array([1.0, 0, 0]), np.array([0.0, 1, 0]), 10)
        self.assertLess(command.saturation_scale, 0.1)
        self.assertTrue(np.all(np.isfinite(command.quaternion)))

    def test_random_normal_requests_preserve_constraints(self):
        rng = np.random.default_rng(1)
        for _ in range(40):
            los = unit([1.0, rng.uniform(-0.2, 0.2), rng.uniform(-0.15, 0.15)])
            frame = pn_frame(los)
            direction = rng.uniform(0, 2*np.pi)
            normal = np.cos(direction)*frame[:, 1]+np.sin(direction)*frame[:, 2]
            self.check_command(los, normal, rng.uniform(0, 0.8))


class PhysicsAndExperimentTests(unittest.TestCase):
    """检查物理平衡、真实门平面穿越、可重复性及积分步长收敛。"""

    def test_hover_equilibrium_and_unit_quaternion(self):
        config = Config()
        plant = Quadrotor(config, np.array([0.0, 0, 3]), np.zeros(3), np.array([1.0, 0, 0, 0]))
        command = Command(config.mass*9.81, plant.q.copy(), np.zeros(3), np.zeros(3), 1, 0, 0)
        for _ in range(240):
            plant.step(command, config.dt)
        np.testing.assert_allclose(plant.p, [0, 0, 3], atol=1e-10)
        np.testing.assert_allclose(plant.v, 0, atol=1e-10)
        self.assertAlmostEqual(np.linalg.norm(plant.q), 1)

    def test_five_motions_cross_the_actual_gate_plane(self):
        # 成功状态之外，还核对末帧确实位于门平面，且误差与保存指标一致。
        for motion in MOTIONS:
            result = simulate(Config(motion=motion, seed=7))
            self.assertTrue(result.metrics["success"], motion)
            relative = result.arrays["position"][-1]-result.arrays["gate"][-1]
            self.assertLess(abs(relative@result.gate_frame[:, 0]), 1e-8)
            self.assertAlmostEqual(np.linalg.norm(relative), result.metrics["miss_distance_m"], places=8)
            norms = np.linalg.norm(result.arrays["quaternion"], axis=1)
            np.testing.assert_allclose(norms, 1, atol=1e-12)

    def test_determinism_and_time_step_convergence(self):
        # 相同种子应完全可重复；步长减半后，穿越时间/误差应保持接近。
        config = Config(motion="stationary", pixel_noise=0)
        first, repeat = simulate(config), simulate(config)
        np.testing.assert_equal(first.arrays["position"], repeat.arrays["position"])
        refined = simulate(replace(config, dt=1/480))
        self.assertLess(abs(first.metrics["miss_distance_m"]-refined.metrics["miss_distance_m"]), 0.01)
        self.assertLess(abs(first.metrics["crossing_time_s"]-refined.metrics["crossing_time_s"]), 0.01)

    def test_gp_ucb_is_finite_and_reward_handles_failures(self):
        x = np.array([[0, 0], [0.5, 0.5], [1, 1]])
        score = gp_ucb(x, np.array([0.0, 1.0, 0.1]), np.array([[0.5, 0.5], [0.2, 0.8]]))
        self.assertEqual(score.shape, (2,))
        self.assertTrue(np.all(np.isfinite(score)))
        self.assertEqual(reward({"crossed": False}, "distance_time"), 0)
        self.assertEqual(reward({"crossed": True, "crossing_time_s": 2, "miss_distance_m": 0.2}, "distance"), 2)


if __name__ == "__main__":
    unittest.main()
