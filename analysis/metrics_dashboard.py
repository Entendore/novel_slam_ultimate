"""
Comprehensive SLAM Metrics Dashboard
"""
import numpy as np
import json
import time
from pathlib import Path
import matplotlib.pyplot as plt

class SLAMMetrics:
    def __init__(self, output_dir="results/metrics"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.metrics = {
            'accuracy': {'ate_rmse': [], 'rpe': []},
            'speed': {'fps': [], 'latency_ms': []},
            'memory': {'ram_mb': [], 'num_keyframes': []},
        }
    
    def update(self, **kwargs):
        for key, value in kwargs.items():
            if key in self.metrics:
                if isinstance(value, dict):
                    for k, v in value.items():
                        self.metrics[key][k].append(v)
                else:
                    self.metrics[key].append(value)
    
    def generate_report(self):
        report = {'summary': {}, 'detailed': self.metrics, 'timestamp': time.strftime('%Y-%m-%d %H:%M:%S')}
        for category, data in self.metrics.items():
            report['summary'][category] = {}
            for metric, values in data.items():
                if values and isinstance(values[0], (int, float)):
                    arr = np.array(values)
                    report['summary'][category][metric] = {
                        'mean': float(np.mean(arr)),
                        'std': float(np.std(arr)),
                        'final': float(arr[-1])
                    }
        with open(self.output_dir / 'metrics_report.json', 'w') as f:
            json.dump(report, f, indent=2)
        print(f"✅ Metrics report saved to {self.output_dir}")
        return report


if __name__ == "__main__":
    metrics = SLAMMetrics()
    for i in range(100):
        metrics.update(
            accuracy={'ate_rmse': 0.1 + np.random.rand() * 0.05},
            speed={'fps': 30 + np.random.rand() * 5, 'latency_ms': 33 + np.random.rand() * 5},
            memory={'num_keyframes': i}
        )
    report = metrics.generate_report()
