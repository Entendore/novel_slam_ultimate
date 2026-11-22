# Novel SLAM Webots Research Framework

A complete, publication-ready SLAM research platform for academic papers (ICRA/IROS/RAL/CoRL).

## 🚀 Features

- **Multi-Sensor Fusion**: 3D LiDAR, IMU, Camera, Depth, GPS
- **Semantic SLAM**: Object recognition and semantic mapping
- **Neural SLAM Ready**: Export data for SplaTAM, iMAP, NICE-SLAM
- **Factor Graph Export**: Visualize in g2o_viewer
- **Deep Place Recognition**: NetVLAD-style loop closure detection
- **Long-term Multi-session SLAM**: Re-localization in previously mapped environments
- **Dynamic Object Filtering**: Remove moving objects from maps
- **Automated Benchmarking**: Generate LaTeX tables for papers
- **Docker Support**: One-command reproducibility
- **ROS2 Integration**: Full launch files and configs

## 📁 Project Structure

```
novel_slam_ultimate/
├── worlds/                    # Webots world files
├── controllers/               # Robot controllers
├── analysis/                  # Analysis and evaluation scripts
├── ros2/                      # ROS2 integration
├── docker/                    # Docker containerization
├── datasets/                  # Generated datasets
├── results/                   # Output results
└── requirements.txt           # Python dependencies
```

## 🛠️ Installation

### Option 1: Local Installation

1. Install Webots R2023b or later: https://cyberbotics.com/

2. Install Python dependencies:
```bash
pip install -r requirements.txt
```

3. (Optional) Install ROS2 and webots_ros2:
```bash
sudo apt install ros-${ROS_DISTRO}-webots-ros2
```

### Option 2: Docker (Recommended)

```bash
cd docker
docker-compose up slam
```

## 🎮 Usage

### Run Basic SLAM Simulation

```bash
webots worlds/slam_arena.wbt
```

### Run Long-term Multi-session SLAM

```bash
# Session 1: Build map
webots worlds/slam_arena.wbt

# Session 2: Re-localize
webots worlds/slam_arena_longterm.wbt
```

### Export Data for Neural SLAM

```bash
python analysis/neural_export.py
```

### Export Factor Graph

```bash
python analysis/factor_graph_export.py
# Visualize with: g2o_viewer results/example_graph.g2o
```

### Run Benchmarking

```bash
python analysis/run_benchmark.py
```

### Generate Metrics Report

```bash
python analysis/metrics_dashboard.py
```

## 📊 Evaluation

### Using evo (Recommended)

```bash
pip install evo

# Evaluate trajectory
evo_traj tum datasets/robot_1_trajectory.csv --ref datasets/ground_truth.txt -p -a

# Calculate ATE
evo_ape tum datasets/ground_truth.txt datasets/robot_1_trajectory.csv -r full -p
```

## 🔬 Injecting Your Novel SLAM Algorithm

Edit `controllers/slam_neural_controller/slam_neural_controller.py` and find this section:

```python
# --- INJECT YOUR NOVEL SLAM ALGORITHM HERE ---
# your_slam.process(point_cloud_2d, imu_data, gps_data)
# estimated_pose = your_slam.get_pose()
# covariance = your_slam.get_covariance()
```

Replace with your algorithm implementation.

## 📝 Generating Paper Figures

All analysis scripts generate publication-ready outputs:

- LaTeX tables in `results/*.tex`
- High-resolution plots in `results/*.png`
- Point clouds in `results/*.pcd`
- Factor graphs in `results/*.g2o`
- Neural SLAM datasets in `datasets/neural/`

## 📚 Citation

If you use this framework in your research, please cite:

```bibtex
@software{novel_slam_webots_2026,
  title = {Novel SLAM Webots Research Framework},
  year = {2026},
  url = {https://github.com/your-repo}
}
```

## 🤝 Contributing

Contributions welcome! Please open an issue or PR.

## 📄 License

MIT License - See LICENSE file for details.
