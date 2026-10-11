"""统一运行入口：python main.py [demo|benchmark|delay|optimize|all]。

demo 展示单次飞行；benchmark 对照论文参数；delay 扫描视觉延迟；
optimize 执行贝叶斯优化；all 依次生成完整实验与可视化结果。
"""

import argparse
from dataclasses import replace
from pathlib import Path
import sys

import matplotlib

from simulation import Config, MOTIONS, PRESETS, simulate


def arguments():
    """解析命令行参数；默认输出目录跟随源码，避免依赖启动位置。"""
    parser = argparse.ArgumentParser(description="Visual Python reproduction of monocular PN gate racing")
    parser.add_argument("task", nargs="?", default="demo", choices=("demo", "benchmark", "delay", "optimize", "all"))
    parser.add_argument("--motion", choices=MOTIONS, default="knot")
    parser.add_argument("--preset", choices=PRESETS, default="distance_time")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--runs", type=int, default=12, help="Runs per motion, per experimental setting")
    parser.add_argument("--iterations", type=int, default=25, help="Total BO evaluations, including 5 initial samples")
    parser.add_argument("--delay", type=float, default=0.0, help="Vision latency in seconds")
    parser.add_argument("--noise", type=float, default=0.4, help="Bounding-box coordinate noise std, pixels")
    parser.add_argument("--wind", type=float, default=0.0, help="World-y wind speed in m/s")
    parser.add_argument("--distance", type=float, default=0.0, help="Initial anchor x distance; zero randomizes 20-30 m")
    parser.add_argument("--radius", type=float, default=1.0, help="Circular gate radius in m")
    parser.add_argument("--k-pn", type=float, help="Override navigation gain")
    parser.add_argument("--gamma", type=float, help="Override view bound, degrees")
    parser.add_argument("--speed-bound", type=float, default=15.0, help="Fixed relative-speed bound, m/s")
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent/"outputs")
    parser.add_argument("--no-show", action="store_true", help="Save artifacts without opening a GUI")
    parser.add_argument("--gif", action="store_true", help="Also save the animated demo as GIF")
    parser.add_argument("--no-animation", action="store_true", help="Only save static demo plots and trajectory data")
    args = parser.parse_args()
    if args.runs < 1 or args.iterations < 1:
        parser.error("--runs and --iterations must be positive")
    return args


def demo(config, directory, animation=True, gif=False):
    """运行一条轨迹，保存数据和图表，可选生成离线动画及 GIF。"""
    from visualization import animate, summary_plot
    result = simulate(config)
    result.save(directory)
    figure = summary_plot(result, directory)
    movie = animate(result, directory, gif=gif) if animation else None
    print(f'Demo {config.motion}: {result.metrics}', flush=True)
    return result, figure, movie


def main():
    """将命令行选项转换为仿真配置，再分发到各实验入口。"""
    args = arguments()
    if args.no_show:
        # 在导入 pyplot 前切换到无窗口后端，支持服务器和自动化运行。
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from experiments import benchmark, delay_sweep, optimize

    gain, gamma = PRESETS[args.preset]
    # 用户显式给出的增益/视角覆盖论文预设，其余参数使用 Config 默认值。
    base = Config(motion=args.motion, seed=args.seed, preset=args.preset,
                  k_pn=gain if args.k_pn is None else args.k_pn,
                  gamma_max_deg=gamma if args.gamma is None else args.gamma,
                  delay=args.delay, pixel_noise=args.noise, wind_y=args.wind,
                  initial_distance=args.distance, gate_radius=args.radius,
                  speed_bound=args.speed_bound)
    args.output.mkdir(parents=True, exist_ok=True)
    keep_alive = []  # 保留动画引用到 plt.show() 结束，避免对象被提前回收。
    if args.task == "demo":
        keep_alive.append(demo(base, args.output/"demo"/args.motion, not args.no_animation, args.gif))
    if args.task == "all":
        # 五种运动都生成静态图；完整模式只为 knot 额外渲染动画，减少耗时。
        for motion in MOTIONS:
            keep_alive.append(demo(replace(base, motion=motion), args.output/"demo"/motion,
                                   not args.no_animation and motion == "knot", args.gif and motion == "knot"))
            if args.no_show:
                plt.close("all")
    if args.task in ("benchmark", "all"):
        benchmark(args.output/"benchmark", args.runs, args.seed+100, base)
    if args.task in ("delay", "all"):
        delay_sweep(args.output/"delay", args.runs, args.seed+300, base)
    if args.task in ("optimize", "all"):
        presets = PRESETS if args.task == "all" else (args.preset,)
        for preset in presets:
            optimize(args.output/"optimization"/preset, preset, args.iterations, args.seed+500, base)
    from reporting import write_report
    # 首页和报告读取实际保存的结果；不重新运行或修改实验数据。
    write_report(args.output)
    print(f'Artifacts saved to {args.output.resolve()}', flush=True)
    if not args.no_show:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
