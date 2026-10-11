"""读取已保存的实验数据，生成离线结果首页与 Markdown 复现报告。"""

import csv
from html import escape
import json
from pathlib import Path

from simulation import MOTIONS
from visualization import LABELS


def write_report(directory):
    """汇总已存在的演示/实验文件；未运行的部分自动跳过，不补造结果。"""
    directory = Path(directory)
    md = ["# 移动门论文 Python 仿真复现结果", "",
          "结果由保存的 JSON/CSV 数据生成。独立 Python 动力学与论文 PX4/Gazebo 环境不同，数值不是论文原始实验数据。", "",
          "## 单次可视化演示", "", "| 门运动 | 结果 | 中心误差 / m | 穿门时间 / s | 最高速度 / m/s |",
          "|---|---|---:|---:|---:|"]
    cards = []
    # 单次演示读取 metrics.json；网页引用原始图片/动画，避免重复复制大文件。
    for motion in MOTIONS:
        folder = directory/"demo"/motion
        if not (folder/"metrics.json").exists():
            continue
        saved = json.loads((folder/"metrics.json").read_text(encoding="utf-8"))
        m = saved["metrics"]
        miss = "--" if m["miss_distance_m"] is None else f'{m["miss_distance_m"]:.3f}'
        time = "--" if m["crossing_time_s"] is None else f'{m["crossing_time_s"]:.3f}'
        md.append(f'| {motion} | {m["status"]} | {miss} | {time} | {m["peak_speed_m_s"]:.3f} |')
        link = f"demo/{motion}"
        movie = f'<a href="{link}/animation.html">播放交互动画</a> · ' if (folder/"animation.html").exists() else ""
        cards.append(f'<article><h2>{escape(LABELS[motion])}</h2>'
                     f'<p>{escape(m["status"])} · 中心误差 {miss} m · 时间 {time} s</p>'
                     f'<p>{movie}<a href="{link}/trajectory.csv">轨迹 CSV</a> · '
                     f'<a href="{link}/metrics.json">指标与配置</a></p>'
                     f'<a href="{link}/summary.png"><img src="{link}/summary.png" alt="{motion} plots"></a></article>')

    csv_path = directory/"benchmark"/"comparison_with_paper.csv"
    if csv_path.exists():
        with csv_path.open(encoding="utf-8") as file:
            rows = list(csv.DictReader(file))
        md += ["", "## 与论文表 I 对照", "",
               "每组默认 12 次；每套参数共 60 次。成功率按门半径 1 m 统计。时间均值包含成功及失败的实际越过门平面记录，未越过平面的运行不进入时间均值。", "",
               "| 参数预设 | 门运动 | Python 成功率 | 论文成功率 | Python 时间 / s | 论文时间 / s |",
               "|---|---|---:|---:|---:|---:|"]
        for r in rows:
            time = "--" if not r["python_crossing_time_s"] else f'{float(r["python_crossing_time_s"]):.3f}'
            md.append(f'| {r["preset"]} | {r["motion"]} | {float(r["python_success_radius_1m"]):.3f} | '
                      f'{float(r["paper_success_radius_1m"]):.3f} | {time} | {float(r["paper_crossing_time_s"]):.3f} |')
        cards.append('<article><h2>随机初始条件：两套论文参数</h2><a href="benchmark/comparison_with_paper.csv">与论文对照 CSV</a>'
                     '<img src="benchmark/benchmark.png" alt="Benchmark results"></article>')

    if (directory/"delay"/"delay.png").exists():
        md += ["", "## 感知延迟", "", "延迟 0/100/200/300 ms，门半径 0.5/1/2 m，静止/快速直线/平面三种运动。各门半径独立仿真，改变半径会同时改变成像和评分。", "",
               "详细结果：`delay/runs.csv`；图：`delay/delay.png`。"]
        cards.append('<article><h2>感知延迟与门尺寸</h2><a href="delay/runs.csv">完整运行 CSV</a>'
                     '<img src="delay/delay.png" alt="Delay robustness"></article>')

    for preset in ("distance_time", "distance"):
        # BO 训练奖励与随机测试成功率分开展示，避免把训练最优当作泛化性能。
        folder = directory/"optimization"/preset
        if not (folder/"best_parameters.json").exists():
            continue
        best = json.loads((folder/"best_parameters.json").read_text(encoding="utf-8"))
        md += ["", f"## 贝叶斯优化：{preset}", "",
               f'最佳 k_PN = {best["k_pn"]:.4f}，视野上限 = {best["gamma_max_deg"]:.4f}°，训练平均奖励 = {best["reward"]:.4f}。', "",
               "使用固定的 8 个训练场景。该训练最优值不代表独立测试集上的泛化成功率。"]
        link = f"optimization/{preset}"
        cards.append(f'<article><h2>贝叶斯优化：{preset}</h2>'
                     f'<p>k = {best["k_pn"]:.4f} · γ = {best["gamma_max_deg"]:.4f}° · 训练奖励 {best["reward"]:.4f}</p>'
                     f'<a href="{link}/history.csv">全部评估记录</a>'
                     f'<img src="{link}/optimization.png" alt="Bayesian optimization"></article>')

    md += ["", "## 复现边界", "",
           "复现了 LOS、比例导航、论文保守视野约束下的闭式推力/姿态解、四旋翼仿真、贝叶斯优化及主要仿真实验。未复现实机、ROS2/PX4/Gazebo、YOLO/Nanotrack训练及论文表 II/III 的外部基线。", "",
           "闭式解约束的是指令姿态相对所测 LOS 的夹角；真实相机角度会受姿态跟踪、测量延迟及近距离穿门影响，图中单独展示。近门检测消失时保持最近有效指令，没有使用真值补充控制。", "",
           "未披露的物理与传感器参数、式 (11) 质量单位及式 (15) 相机夹角处理见项目 README.md。"]
    (directory/"reproduction_report.md").write_text("\n".join(md)+"\n", encoding="utf-8")
    html = ('<!doctype html><html lang="zh-CN"><meta charset="utf-8">'
            '<meta name="viewport" content="width=device-width,initial-scale=1"><title>移动门论文复现</title>'
            '<style>body{max-width:1250px;margin:35px auto;padding:0 20px;background:#f6f8fb;'
            'font-family:system-ui,sans-serif;color:#25384a;line-height:1.6}'
            'article{background:white;border:1px solid #dce4eb;border-radius:12px;padding:20px;margin:25px 0}'
            'img{display:block;width:100%;height:auto;margin-top:15px}a{color:#1686b0}h1{line-height:1.2}</style>'
            '<h1>Flying through Moving Gates<br>without Full State Estimation</h1>'
            '<p>完整 Python 仿真与可视化结果 · 所有数据由代码实际运行生成</p>'
            '<p>采用独立四旋翼与单目相机模型；参数差异会影响与论文的数值对照。'
            '<a href="reproduction_report.md">复现报告</a></p>'+"".join(cards)+'</html>')
    (directory/"index.html").write_text(html, encoding="utf-8")
