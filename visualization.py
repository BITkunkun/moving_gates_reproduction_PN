"""可视化：单次诊断图、四视图离线动画，以及随机实验/延迟/优化曲线。

英文图注沿用论文术语；内部注释说明数组含义与图形更新方式。
"""

from pathlib import Path
import re

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle, Rectangle
import numpy as np

from geometry import quat_matrix
from simulation import Camera


# 统一配色：蓝色无人机/检测，橙色门，绿色 PN 法向加速度。
BLUE, ORANGE, GREEN, INK = "#1686b0", "#e88b38", "#238b68", "#25384a"
LABELS = {"stationary": "Stationary", "linear_slow": "Linear slow (2.5 m/s)",
          "linear_fast": "Linear fast (5 m/s)", "planar": "Planar", "knot": "3D knot"}


def offline_controls(content):
    """用本地字符替换播放器的在线图标字体，使 HTML 无需网络即可操作。"""
    content = re.sub(r"<link\b[^>]*>", "", content, flags=re.IGNORECASE)
    glyphs = {
        "fa-minus": "&#8722;", "fa-fast-backward": "|&#9664;", "fa-step-backward": "&#9664;|",
        "fa-play fa-flip-horizontal": "&#9664;", "fa-pause": "||", "fa-play": "&#9654;",
        "fa-step-forward": "|&#9654;", "fa-fast-forward": "&#9654;|", "fa-plus": "+",
    }
    for name, glyph in glyphs.items():
        content = content.replace(f'<i class="fa {name}"></i>', glyph)
    return content


def style():
    """设置全项目图表使用的字体、颜色和背景。"""
    plt.rcParams.update({
        "font.family": "DejaVu Sans", "font.size": 9, "axes.titlesize": 11,
        "axes.labelcolor": INK, "text.color": INK, "axes.edgecolor": "#c7d3dc",
        "axes.spines.top": False, "axes.spines.right": False,
        "grid.color": "#dce4eb", "grid.alpha": 0.65,
        "figure.facecolor": "#f6f8fb", "axes.facecolor": "white",
        "savefig.facecolor": "#f6f8fb", "legend.frameon": False,
    })


def ring(result, index, points=97):
    """按固定门平面的两个切向基生成 index 时刻的圆环边界点。"""
    angles = np.linspace(0, 2*np.pi, points)
    return result.arrays["gate"][index]+result.config.gate_radius*(
        np.cos(angles)[:, None]*result.gate_frame[:, 1]
        + np.sin(angles)[:, None]*result.gate_frame[:, 2])


def title(result):
    """从实际评分生成状态、误差、时间和速度标题；无穿越时显示 --。"""
    m = result.metrics
    miss = "--" if m["miss_distance_m"] is None else f'{m["miss_distance_m"]:.3f} m'
    time = "--" if m["crossing_time_s"] is None else f'{m["crossing_time_s"]:.2f} s'
    return (f'{LABELS[result.config.motion]}  |  {m["status"].upper()}  |  '
            f'center error {miss}  |  crossing {time}  |  peak {m["peak_speed_m_s"]:.2f} m/s')


def setup_3d(ax, result):
    """用轨迹及圆环范围设置三维视角，预留坐标标签与门边界的显示空间。"""
    p, g = result.arrays["position"], result.arrays["gate"]
    all_points = np.concatenate((p, g, ring(result, -1)))
    minimum, maximum = all_points.min(axis=0), all_points.max(axis=0)
    padding = np.maximum(0.1*(maximum-minimum), [1.0, 1.5, 0.8])
    ax.set(xlim=(minimum[0]-padding[0], maximum[0]+padding[0]),
           ylim=(minimum[1]-padding[1], maximum[1]+padding[1]),
           zlim=(max(0, minimum[2]-padding[2]), maximum[2]+padding[2]),
           xlabel="x [m]", ylabel="y [m]", zlabel="z [m]")
    ax.set_box_aspect((2.0, 1.0, 0.8))
    ax.view_init(elev=24, azim=-63)
    ax.plot(*g.T, color=ORANGE, ls="--", alpha=0.65, label="Gate center")


def setup_camera(ax, camera):
    """设置像素图：v 轴向下，虚线圆表示论文采用的视线可见锥。"""
    ax.set(xlim=(0, camera.width), ylim=(camera.height, 0), aspect="equal",
           xlabel="u [pixel]", ylabel="v [pixel]", title="Forward monocular camera")
    ax.add_patch(Circle((camera.cx, camera.cy), camera.height/2, fill=False,
                        color="#a9bbc8", ls="--", lw=1))
    ax.plot(camera.cx, camera.cy, "+", color=INK, ms=9)


def projected_ring(result, index, camera):
    """用当前姿态投影真实门边界；只用于绘图，不反馈给控制器。"""
    return camera.project(ring(result, index), result.arrays["position"][index],
                          quat_matrix(result.arrays["quaternion"][index]))


def summary_plot(result, directory):
    """保存六面板诊断图：三维、俯视、相机、速度/推力、视角及法向限幅。"""
    style()
    a, c = result.arrays, result.config
    # a 保存时间序列数组，c 保存配置；各面板使用同一次仿真的数据。
    fig = plt.figure(figsize=(15, 8.6), layout="constrained")
    grid = fig.add_gridspec(2, 3)
    fig.suptitle("PN-informed flight through moving gates\n"+title(result), fontsize=15)
    ax = fig.add_subplot(grid[0, 0], projection="3d")
    setup_3d(ax, result)
    ax.plot(*a["position"].T, color=BLUE, lw=2, label="Quadrotor")
    ax.plot(*ring(result, -1).T, color=ORANGE, lw=2)
    ax.set_title("3D trajectory and gate at crossing")
    ax.legend(loc="upper left", fontsize=8)

    ax = fig.add_subplot(grid[0, 1])
    ax.plot(a["position"][:, 0], a["position"][:, 1], color=BLUE, lw=2, label="Quadrotor")
    ax.plot(a["gate"][:, 0], a["gate"][:, 1], color=ORANGE, ls="--", lw=2, label="Gate center")
    ax.scatter(*a["position"][-1, :2], color=GREEN, s=45, zorder=4)
    select = np.arange(0, len(a["time"]), max(1, len(a["time"])//10))
    normals = a["normal_acceleration"][select]
    ax.quiver(a["position"][select, 0], a["position"][select, 1], normals[:, 0], normals[:, 1],
              color=GREEN, angles="xy", scale_units="xy", scale=4, width=0.004)
    ax.set(xlabel="x [m]", ylabel="y [m]", title="Top view / normal acceleration arrows")
    ax.grid(True)
    ax.legend(loc="upper left")

    camera = Camera(c.camera_half_fov_deg)
    ax = fig.add_subplot(grid[0, 2])
    setup_camera(ax, camera)
    index = min(len(a["time"])-2, int(0.75*len(a["time"])))
    # 选接近门但尚未穿越的帧，便于观察当前门投影与已接收检测框的差异。
    pixels = projected_ring(result, index, camera)
    ax.plot(*pixels.T, color=ORANGE, lw=2, label="Projected gate")
    bbox = a["bbox"][index]
    if np.all(np.isfinite(bbox)):
        ax.add_patch(Rectangle(bbox[:2], bbox[2]-bbox[0], bbox[3]-bbox[1],
                               fill=False, color=BLUE, ls="--", lw=1.5))
        ax.plot(*(bbox[:2]+bbox[2:])/2, "o", color=BLUE, ms=4)
    ax.text(0.03, 0.97, f't = {a["time"][index]:.2f} s\nBlue: delivered bounding box',
            transform=ax.transAxes, va="top", fontsize=8)

    ax = fig.add_subplot(grid[1, 0])
    ax.plot(a["time"], np.linalg.norm(a["velocity"], axis=1), color=BLUE, label="Speed")
    ax.set(xlabel="Time [s]", ylabel="Speed [m/s]", title="Speed and thrust")
    ax.grid(True)
    secondary = ax.twinx()
    secondary.plot(a["time"], a["thrust"], color=ORANGE, alpha=0.8, label="Thrust")
    secondary.set_ylabel("Thrust [N]", color=ORANGE)
    ax.legend(loc="upper left")
    secondary.legend(loc="lower right")

    ax = fig.add_subplot(grid[1, 1])
    # 指令满足所测视线约束，不保证延迟/跟踪误差下的真实相机视角始终可见。
    ax.plot(a["time"], a["gamma_deg"], color=ORANGE, label="Actual camera / true LOS")
    ax.plot(a["time"], a["command_gamma_deg"], color=BLUE, label="Command / measured LOS")
    ax.axhline(c.gamma_max_deg, color=BLUE, ls=":", label="Command bound")
    ax.axhline(c.camera_half_fov_deg, color="#b34651", ls="--", label="Camera half-FOV")
    ax.set(xlabel="Time [s]", ylabel="Angle [deg]", title="View constraint and tracking error")
    ax.grid(True)
    ax.legend(fontsize=8, loc="upper left")

    ax = fig.add_subplot(grid[1, 2])
    ax.plot(a["time"], np.linalg.norm(a["requested_normal"], axis=1), color="#8ebda9", ls="--", label="Requested PN")
    ax.plot(a["time"], np.linalg.norm(a["normal_acceleration"], axis=1), color=GREEN, label="Feasible PN")
    ax.set(xlabel="Time [s]", ylabel="Acceleration [m/s^2]", title="PN acceleration / feasibility saturation")
    ax.grid(True)
    ax.legend(loc="upper left")
    path = Path(directory)/"summary.png"
    fig.savefig(path, dpi=150)
    return fig


def animate(result, directory, gif=False):
    """生成三维/俯视/相机/遥测同步动画，保存可逐帧播放的 HTML 和可选 GIF。"""
    style()
    a, c = result.arrays, result.config
    camera = Camera(c.camera_half_fov_deg)
    fig = plt.figure(figsize=(12.8, 8.2), layout="constrained")
    grid = fig.add_gridspec(2, 2, height_ratios=[1.05, 1])
    fig.suptitle("Monocular PN guidance | "+LABELS[c.motion], fontsize=16)
    ax3 = fig.add_subplot(grid[0, 0], projection="3d")
    setup_3d(ax3, result)
    drone_trail, = ax3.plot([], [], [], color=BLUE, lw=2)
    drone_dot, = ax3.plot([], [], [], "o", color=BLUE, ms=6)
    gate_ring, = ax3.plot([], [], [], color=ORANGE, lw=2)
    body_x, = ax3.plot([], [], [], color=INK, lw=3)
    body_y, = ax3.plot([], [], [], color=INK, lw=3)
    ax3.set_title("3D flight / moving gate")

    xy = fig.add_subplot(grid[0, 1])
    xy.plot(a["gate"][:, 0], a["gate"][:, 1], color=ORANGE, ls="--", alpha=0.5)
    xy_trail, = xy.plot([], [], color=BLUE, lw=2, label="Quadrotor")
    xy_drone, = xy.plot([], [], "o", color=BLUE)
    xy_gate, = xy.plot([], [], "o", color=ORANGE, label="Gate center")
    xy_los, = xy.plot([], [], color=GREEN, ls=":", lw=1.5)
    combined = np.concatenate((a["position"], a["gate"]))
    xy.set(xlim=(combined[:, 0].min()-1, combined[:, 0].max()+2),
           ylim=(combined[:, 1].min()-2, combined[:, 1].max()+2),
           xlabel="x [m]", ylabel="y [m]", title="Top view / true LOS (evaluation only)")
    xy.grid(True)
    xy.legend(loc="upper left")

    cam_ax = fig.add_subplot(grid[1, 0])
    setup_camera(cam_ax, camera)
    camera_ring, = cam_ax.plot([], [], color=ORANGE, lw=2)
    bbox_patch = Rectangle((0, 0), 0, 0, fill=False, color=BLUE, ls="--", lw=1.5)
    cam_ax.add_patch(bbox_patch)
    cam_text = cam_ax.text(0.025, 0.97, "", transform=cam_ax.transAxes, va="top", fontsize=8)

    telemetry = fig.add_subplot(grid[1, 1])
    speed = np.linalg.norm(a["velocity"], axis=1)
    speed_line, = telemetry.plot([], [], color=BLUE, lw=2, label="Speed [m/s]")
    telemetry.set(xlim=(0, max(a["time"][-1], 0.1)), ylim=(0, max(speed)*1.25),
                  xlabel="Time [s]", ylabel="Speed [m/s]", title="Live telemetry")
    telemetry.grid(True)
    secondary = telemetry.twinx()
    gamma_line, = secondary.plot([], [], color=ORANGE, lw=1.5)
    secondary.axhline(c.camera_half_fov_deg, color="#b34651", ls="--", lw=1)
    secondary.set(ylim=(0, 100), ylabel="Camera / true LOS [deg]")
    telemetry.legend(loc="lower right")
    live_text = telemetry.text(0.035, 0.95, "", transform=telemetry.transAxes, va="top", fontsize=10,
                               bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.9})

    def update(i):
        """更新第 i 个采样时刻；复用已有绘图对象，避免每帧重建坐标轴。"""
        p, g = a["position"][i], a["gate"][i]
        rotation = quat_matrix(a["quaternion"][i])
        drone_trail.set_data_3d(*a["position"][:i+1].T)
        drone_dot.set_data_3d(*p[:, None])
        gate_ring.set_data_3d(*ring(result, i).T)
        for artist, direction in ((body_x, rotation[:, 0]), (body_y, rotation[:, 1])):
            artist.set_data_3d(*np.array([p-0.3*direction, p+0.3*direction]).T)
        xy_trail.set_data(*a["position"][:i+1, :2].T)
        xy_drone.set_data([p[0]], [p[1]])
        xy_gate.set_data([g[0]], [g[1]])
        xy_los.set_data([p[0], g[0]], [p[1], g[1]])
        camera_ring.set_data(*projected_ring(result, i, camera).T)
        # 圆环是当前图像，蓝框来自延迟后交付的检测，因此二者可以错开。
        bbox = a["bbox"][i]
        bbox_patch.set_visible(bool(np.all(np.isfinite(bbox))))
        if np.all(np.isfinite(bbox)):
            bbox_patch.set_xy(bbox[:2])
            bbox_patch.set_width(bbox[2]-bbox[0])
            bbox_patch.set_height(bbox[3]-bbox[1])
        cam_text.set_text(f'Capture age: {a["measurement_age"][i]:.3f} s\nBlue: delayed detection / orange: current gate')
        speed_line.set_data(a["time"][:i+1], speed[:i+1])
        gamma_line.set_data(a["time"][:i+1], a["gamma_deg"][:i+1])
        final = "\n"+result.metrics["status"].upper() if i == len(a["time"])-1 else ""
        live_text.set_text(f't = {a["time"][i]:.2f} s    speed = {speed[i]:.2f} m/s\n'
                           f'thrust = {a["thrust"][i]:.2f} N    LOS rate = {a["los_rate"][i]:.3f} rad/s\n'
                           f'PN scale = {a["saturation_scale"][i]:.2f}{final}')
        return drone_trail, drone_dot, gate_ring, body_x, body_y, xy_trail, camera_ring

    # 最多渲染 180 帧，控制 HTML 体积；用仿真总时长确定正常播放帧率。
    frames = np.unique(np.linspace(0, len(a["time"])-1, min(180, len(a["time"]))).astype(int))
    fps = len(frames)/max(float(a["time"][-1]), 0.1)
    animation = FuncAnimation(fig, update, frames=frames, interval=1000/fps, blit=False, repeat=False)
    plt.rcParams["animation.embed_limit"] = 80
    content = offline_controls(animation.to_jshtml(fps=fps, default_mode="once"))
    # 图片帧嵌入 HTML，播放器无需后端；GIF 用 Pillow 输出，不依赖 FFmpeg。
    html = ('<!doctype html><html lang="en"><meta charset="utf-8"><title>Moving gate PN simulation</title>'
            '<style>.animation{max-width:100%}.animation>img{max-width:100%;height:auto}'
            '.anim-buttons button{height:28px;font-size:16px;color:#25384a}</style>'
            '<body style="margin:20px;background:#f6f8fb;font-family:Arial;color:#25384a">'
            '<h2>Flying through Moving Gates without Full State Estimation</h2>'
            '<p>Independent Python simulation. Use the playback controls below.</p>'
            f'<p>{title(result)}</p>'+content+'</body></html>')
    directory = Path(directory)
    (directory/"animation.html").write_text(html, encoding="utf-8")
    if gif:
        animation.save(directory/"animation.gif", writer=PillowWriter(fps=min(fps, 30)), dpi=85)
    return fig, animation


def benchmark_plot(rows, path):
    """比较两套论文参数在五种运动上的成功率、穿越时间和中心误差。"""
    from simulation import MOTIONS
    style()
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), layout="constrained")
    fig.suptitle("Measured Python reproduction results | randomized initial conditions", fontsize=15)
    x = np.arange(len(MOTIONS))
    for j, (preset, color) in enumerate((("distance_time", BLUE), ("distance", ORANGE))):
        success, times, errors = [], [], []
        for motion in MOTIONS:
            group = [r for r in rows if r["preset"] == preset and r["motion"] == motion]
            success.append(np.mean([r["success"] for r in group]))
            crossing = [r for r in group if r["crossed"]]
            times.append(np.mean([r["crossing_time_s"] for r in crossing]) if crossing else np.nan)
            errors.append(np.mean([r["miss_distance_m"] for r in crossing]) if crossing else np.nan)
        for ax, values in zip(axes, (success, times, errors)):
            ax.bar(x+(j-0.5)*0.36, values, width=0.34, color=color, label=preset.replace("_", " + "))
    for ax, label in zip(axes, ("Success fraction (radius 1 m)", "Mean crossing time [s]", "Mean center error [m]")):
        ax.set_ylabel(label)
        ax.set_xticks(x, ["Static", "Slow", "Fast", "Planar", "Knot"])
        ax.grid(axis="y")
        ax.set_axisbelow(True)
        ax.legend(fontsize=8)
    axes[0].set_ylim(0, 1.1)
    fig.savefig(path, dpi=160)
    return fig


def delay_plot(rows, path):
    """按实际门半径汇总成功率，将延迟从秒转换为毫秒绘制。"""
    style()
    fig, ax = plt.subplots(figsize=(8.5, 5.0), layout="constrained")
    for radius, color, marker in ((0.5, BLUE, "+"), (1.0, ORANGE, "x"), (2.0, GREEN, "o")):
        delays = sorted({r["delay_s"] for r in rows})
        success = [np.mean([r["success"] for r in rows if r["delay_s"] == d and r["radius_m"] == radius]) for d in delays]
        ax.plot(np.array(delays)*1000, success, marker=marker, color=color, label=f"Radius {radius:g} m")
    ax.set(xlabel="Bounding-box latency [ms]", ylabel="Success fraction", ylim=(-0.04, 1.06),
           title="Perception-delay robustness | static, fast linear, planar")
    ax.grid(True)
    ax.legend()
    fig.savefig(path, dpi=160)
    return fig


def optimization_plot(history, path):
    """绘制每次奖励、历史最优奖励，以及已评估参数在搜索区间内的分布。"""
    style()
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), layout="constrained")
    iterations = np.arange(1, len(history)+1)
    rewards = np.array([r["reward"] for r in history])
    axes[0].plot(iterations, rewards, "o", color="#abc0cd", label="Evaluation")
    axes[0].plot(iterations, np.maximum.accumulate(rewards), color=BLUE, lw=2, label="Best so far")
    axes[0].set(xlabel="Evaluation", ylabel="Mean reward, Eq. (16)", title="Matérn-5/2 GP + UCB optimization")
    axes[0].legend()
    scatter = axes[1].scatter([r["k_pn"] for r in history], [r["gamma_max_deg"] for r in history],
                             c=rewards, cmap="viridis", s=60, edgecolor="white")
    axes[1].set(xlabel="Navigation gain k_PN", ylabel="View bound [deg]", title="Evaluated parameters",
                xlim=(0.25, 3.05), ylim=(4.5, 30.5))
    fig.colorbar(scatter, ax=axes[1], label="Mean reward")
    for ax in axes:
        ax.grid(True)
    fig.savefig(path, dpi=160)
    return fig
