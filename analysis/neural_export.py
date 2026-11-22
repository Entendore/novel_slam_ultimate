"""
Export Webots SLAM data for Neural SLAM frameworks (SplaTAM, iMAP, NICE-SLAM)
"""
import os
import json
import numpy as np
import cv2
from pathlib import Path

class NeuralSLAMExporter:
    def __init__(self, output_dir="datasets/neural", format="replica"):
        self.output_dir = Path(output_dir)
        self.format = format
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        if format == "replica":
            (self.output_dir / "results").mkdir(exist_ok=True)
            (self.output_dir / "traj").mkdir(exist_ok=True)
    
    def export_frame(self, timestamp, rgb_image, depth_image, pose_4x4, intrinsics):
        if self.format == "replica":
            frame_id = int(timestamp * 10)
            cv2.imwrite(str(self.output_dir / f"results/{frame_id:06d}.jpg"), rgb_image)
            np.save(str(self.output_dir / f"traj/{frame_id:06d}.npy"), pose_4x4)
    
    def finalize(self):
        if self.format == "replica":
            camera_info = {
                "w": 640, "h": 480,
                "fx": 525.0, "fy": 525.0,
                "cx": 320.0, "cy": 240.0,
                "png_depth_scale": 1000.0,
                "camera_type": "pinhole"
            }
            with open(self.output_dir / "camera.json", 'w') as f:
                json.dump(camera_info, f, indent=2)
        print(f"✅ Neural SLAM dataset exported to {self.output_dir}")


if __name__ == "__main__":
    exporter = NeuralSLAMExporter(format="replica")
    for i in range(10):
        t = i * 0.1
        rgb = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        pose = np.eye(4)
        pose[:3, 3] = [t, 0, 0]
        exporter.export_frame(t, rgb, None, pose, [525.0, 525.0, 320.0, 240.0])
    exporter.finalize()
