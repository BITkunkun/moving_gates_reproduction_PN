"""实验入口：表 I 参数对照、图 5 延迟扫描，以及 GP/UCB 贝叶斯优化。"""

import csv
from dataclasses import asdict, replace
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import cho_factor, cho_solve, solve_triangular
from scipy.optimize import minimize

from simulation import Config, MOTIONS, PRESETS, simulate
from visualization import benchmark_plot, delay_plot, optimization_plot


# 论文原始表 I 的参考数值，仅用于对照；仿真结果由 simulate 实际计算。
PAPER_TABLE_I = {
    "distance_time": {"success": [1.0, 1.0, 0.67, 1.0, 1.0], "time": [4.11, 4.03, 5.41, 4.01, 4.03]},
    "distance": {"success": [1.0, 1.0, 0.92, 0.50, 1.0], "time": [5.28, 5.35, 5.80, 5.76, 5.41]},
}


def write_csv(path, rows):
    """把逐次运行记录保存为 CSV，字段顺序沿用首条记录。"""
    with Path(path).open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def save_settings(directory, base, **settings):
    """保存公共仿真配置、随机种子及实验设置，便于复查运行条件。"""
    (Path(directory)/"settings.json").write_text(
        json.dumps({"base_config": asdict(base), **settings}, indent=2), encoding="utf-8")


def reward(metrics, preset):
    """论文式 (16)：按穿越时间 t 和中心误差 d 计算非负奖励。"""
    if not metrics["crossed"]:
        return 0.0
    # ct 奖励较短时间，cd 惩罚中心误差，offset 保持距离奖励的有效范围。
    ct, cd, offset = (10.0, 3.0, 0.0) if preset == "distance_time" else (0.0, 5.0, 3.0)
    return max(0.0, ct/metrics["crossing_time_s"]-cd*metrics["miss_distance_m"]+offset)


def benchmark(directory, runs=12, seed=100, base=Config()):
    """两套论文参数 × 五种门运动；每种运动默认运行 12 个随机场景。"""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    base = replace(base, gate_radius=1.0)
    save_settings(directory, base, runs_per_motion=runs, seed_start=seed,
                  presets=PRESETS, radius_2m_is_scoring_threshold_only=True)
    rows, summary = [], []
    for preset, (k_pn, gamma) in PRESETS.items():
        for motion in MOTIONS:
            for i in range(runs):
                config = replace(base, motion=motion, seed=seed+i, preset=preset,
                                 k_pn=k_pn, gamma_max_deg=gamma)
                metrics = simulate(config).metrics
                rows.append(metrics)
            group = rows[-runs:]
            # 时间/误差均值包括越过门平面但错过门洞的运行；超时不计入均值。
            crossings = [r for r in group if r["crossed"]]
            motion_index = MOTIONS.index(motion)
            summary.append({
                "preset": preset, "motion": motion, "runs": runs,
                "python_success_radius_1m": float(np.mean([r["crossed"] and r["miss_distance_m"] <= 1 for r in group])),
                # 2 m 列只改变评分阈值；这批轨迹的实际成像半径仍为 1 m。
                "python_success_threshold_2m": float(np.mean([r["crossed"] and r["miss_distance_m"] <= 2 for r in group])),
                "python_crossing_time_s": float(np.mean([r["crossing_time_s"] for r in crossings])) if crossings else None,
                "python_center_error_m": float(np.mean([r["miss_distance_m"] for r in crossings])) if crossings else None,
                "paper_success_radius_1m": PAPER_TABLE_I[preset]["success"][motion_index],
                "paper_crossing_time_s": PAPER_TABLE_I[preset]["time"][motion_index],
            })
            print(f'Benchmark {preset:14s} {motion:12s}: '
                  f'{sum(r["success"] for r in group)}/{runs} passed', flush=True)
    write_csv(directory/"runs.csv", rows)
    write_csv(directory/"comparison_with_paper.csv", summary)
    (directory/"summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8")
    fig = benchmark_plot(rows, directory/"benchmark.png")
    plt.close(fig)
    return rows


def delay_sweep(directory, runs=12, seed=300, base=Config()):
    """扫描四档延迟和三种门半径，统计静止/快直线/平面运动成功率。"""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    save_settings(directory, base, runs_per_motion=runs, seed_start=seed,
                  delays_s=[0.0, 0.1, 0.2, 0.3], actual_gate_radii_m=[0.5, 1.0, 2.0])
    rows = []
    for delay in (0.0, 0.1, 0.2, 0.3):
        for radius in (0.5, 1.0, 2.0):
            for motion in ("stationary", "linear_fast", "planar"):
                for i in range(runs):
                    # 此处半径同时影响相机成像和成功判定，必须分别进行仿真。
                    metrics = simulate(replace(base, motion=motion, seed=seed+i,
                                               delay=delay, gate_radius=radius)).metrics
                    rows.append({"delay_s": delay, "radius_m": radius, **metrics})
            group = rows[-3*runs:]
            print(f'Delay {delay*1000:3.0f} ms / radius {radius:g} m: '
                  f'{sum(r["success"] for r in group)}/{len(group)} passed', flush=True)
    write_csv(directory/"runs.csv", rows)
    fig = delay_plot(rows, directory/"delay.png")
    plt.close(fig)
    return rows


def matern52(x, y, lengths):
    """Matérn-5/2 核；lengths 为两个参数方向的相关长度尺度。"""
    r = np.sqrt(np.sum(((x[:, None, :]-y[None, :, :])/lengths)**2, axis=-1))
    root5r = np.sqrt(5)*r
    return (1+root5r+5*r*r/3)*np.exp(-root5r)


def gp_ucb(x, y, candidates):
    """拟合 Matérn-5/2 高斯过程，返回候选点的 UCB = 均值 + 2×标准差。"""
    # 标准化奖励，避免不同奖励量级使核参数拟合不稳定。
    mean, deviation = np.mean(y), max(float(np.std(y)), 0.05)
    normalized = (y-mean)/deviation

    def likelihood(log_lengths):
        """负对数边际似然；在对数空间优化，保证核长度始终为正。"""
        covariance = matern52(x, x, np.exp(log_lengths))+1e-5*np.eye(len(x))
        # 对角小项增加数值稳定性；Cholesky 分解避免显式求协方差矩阵逆。
        factor = cho_factor(covariance, lower=True)
        weights = cho_solve(factor, normalized)
        return 0.5*normalized@weights+np.log(np.diag(factor[0])).sum()+0.5*len(x)*np.log(2*np.pi)

    fit = minimize(likelihood, np.log([0.3, 0.3]), method="L-BFGS-B",
                   bounds=[(np.log(0.03), np.log(2.0))]*2)
    lengths = np.exp(fit.x)
    covariance = matern52(x, x, lengths)+1e-5*np.eye(len(x))
    factor = cho_factor(covariance, lower=True)
    cross = matern52(x, candidates, lengths)
    mu = cross.T @ cho_solve(factor, normalized)
    projected = solve_triangular(factor[0], cross, lower=True)
    sigma = np.sqrt(np.maximum(1-np.sum(projected**2, axis=0), 0))
    # 用不确定度鼓励探索，再把采集函数换回原始奖励量级。
    return mean+deviation*(mu+2*sigma)


def optimize(directory, preset="distance_time", iterations=25, seed=500, base=Config()):
    """在论文搜索区间优化 PN 增益与视野上限；每个参数点评估 8 个场景。"""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    save_settings(directory, base, preset=preset, evaluations=iterations, seed=seed,
                  cases_per_evaluation=8, initial_samples=min(iterations, 5),
                  ucb_sigma_multiplier=2, candidate_pool=2048)
    rng = np.random.default_rng(seed)
    lower, width = np.array([0.3, 5.0]), np.array([2.7, 25.0])
    # GP 输入归一化到 [0, 1]²；实际参数 = lower + width * normalized。
    initial = rng.uniform(size=(5, 2))
    initial[0] = (np.asarray(PRESETS[preset])-lower)/width
    x, y, history = [], [], []
    for iteration in range(iterations):
        if iteration < len(initial):
            normalized = initial[iteration]
        else:
            # 从随机候选池选最大 UCB 点；其余初始点用于建立首个 GP 模型。
            candidates = rng.uniform(size=(2048, 2))
            score = gp_ucb(np.asarray(x), np.asarray(y), candidates)
            normalized = candidates[np.argmax(score)]
        k_pn, gamma = lower+width*normalized
        rewards = []
        for motion in MOTIONS[1:]:
            for run in range(2):
                # 每次复用相同的八个训练场景，参数间奖励差异才具有可比性。
                result = simulate(replace(base, motion=motion, preset=preset, seed=seed+100+run,
                                          k_pn=float(k_pn), gamma_max_deg=float(gamma)))
                rewards.append(reward(result.metrics, preset))
        score = float(np.mean(rewards))
        x.append(normalized)
        y.append(score)
        history.append({"evaluation": iteration+1, "k_pn": float(k_pn),
                        "gamma_max_deg": float(gamma), "reward": score})
        print(f'BO {preset} {iteration+1:02d}/{iterations}: k={k_pn:.3f}, '
              f'gamma={gamma:.2f}, reward={score:.3f}', flush=True)
    best = dict(history[int(np.argmax(y))])
    # 这里只记录训练集最优点；独立场景的成功率需要另行评估。
    best["preset"] = preset
    best["training_cases_per_evaluation"] = 8
    best["seed"] = seed
    write_csv(directory/"history.csv", history)
    (directory/"best_parameters.json").write_text(json.dumps(best, indent=2), encoding="utf-8")
    fig = optimization_plot(history, directory/"optimization.png")
    plt.close(fig)
    return best
