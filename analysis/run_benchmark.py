"""
Automated Benchmarking & LaTeX Generator
"""
import subprocess
import os
import re
import shutil

WORLD_FILE = "worlds/slam_arena.wbt"
RESULTS_DIR = "results/benchmark"

configs = [
    {"name": "Clear_NoLowNoise", "fog": 30.0, "lidar_noise": 0.02, "light": 0.8},
    {"name": "HeavyFog", "fog": 5.0, "lidar_noise": 0.02, "light": 0.8},
    {"name": "HighSensorNoise", "fog": 30.0, "lidar_noise": 0.15, "light": 0.8},
    {"name": "LowLight", "fog": 30.0, "lidar_noise": 0.02, "light": 0.1}
]

def modify_world_file(fog_range, lidar_noise, light_intensity):
    with open(WORLD_FILE, 'r') as f:
        content = f.read()
    content = re.sub(r'visibilityRange [\d\.]+', f'visibilityRange {fog_range}', content)
    content = re.sub(r'noise [\d\.]+\n\s+enabled TRUE', f'noise {lidar_noise}\n      enabled TRUE', content)
    content = re.sub(r'intensity [\d\.]+', f'intensity {light_intensity}', content)
    with open(WORLD_FILE, 'w') as f:
        f.write(content)

def generate_latex_table(results):
    latex = """
\\begin{table}[h]
\\centering
\\caption{SLAM Robustness Evaluation}
\\label{tab:robustness}
\\begin{tabular}{|l|c|c|c|}
\\hline
\\textbf{Scenario} & \\textbf{Fog Range} & \\textbf{LiDAR Noise} & \\textbf{ATE RMSE (m)} \\\\
\\hline
"""
    for row in results:
        latex += row + "\n"
    latex += "\\hline\n\\end{tabular}\n\\end{table}"
    with open(f"{RESULTS_DIR}/results_table.tex", 'w') as f:
        f.write(latex)
    print(f"✅ LaTeX table saved to {RESULTS_DIR}/results_table.tex")

if __name__ == "__main__":
    os.makedirs(RESULTS_DIR, exist_ok=True)
    print("Benchmark framework ready. Run with: python analysis/run_benchmark.py")
