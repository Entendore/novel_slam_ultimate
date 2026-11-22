"""
Deep Place Recognition for Loop Closure Detection
"""
import numpy as np
from collections import deque

class PlaceRecognition:
    def __init__(self, embedding_dim=128):
        self.embedding_dim = embedding_dim
        self.descriptors = []
        self.timestamps = []
        self.poses = []
        self.recent_places = deque(maxlen=50)
        self.similarity_threshold = 0.7
        self.temporal_threshold = 25
    
    def extract_descriptor(self, image):
        # Simulated descriptor extraction
        return np.random.randn(self.embedding_dim)
    
    def add_place(self, timestamp, pose, descriptor):
        self.descriptors.append(descriptor)
        self.timestamps.append(timestamp)
        self.poses.append(pose)
        self.recent_places.append(len(self.descriptors) - 1)
    
    def query_place(self, descriptor):
        if len(self.descriptors) < self.temporal_threshold:
            return None, 0.0
        candidates = []
        for idx, ref_desc in enumerate(self.descriptors):
            time_diff = len(self.descriptors) - idx
            if time_diff > self.temporal_threshold:
                similarity = np.dot(descriptor, ref_desc) / (
                    np.linalg.norm(descriptor) * np.linalg.norm(ref_desc) + 1e-8
                )
                if similarity > self.similarity_threshold:
                    candidates.append((idx, similarity))
        if candidates:
            best_idx, best_sim = max(candidates, key=lambda x: x[1])
            return best_idx, best_sim
        return None, 0.0
    
    def detect_loop_closure(self, current_timestamp, current_pose, image):
        descriptor = self.extract_descriptor(image)
        match_idx, similarity = self.query_place(descriptor)
        self.add_place(current_timestamp, current_pose, descriptor)
        if match_idx is not None:
            return True, match_idx, self.poses[match_idx], similarity
        return False, None, None, 0.0


if __name__ == "__main__":
    pr = PlaceRecognition()
    for i in range(100):
        t = i * 0.1
        pose = np.array([i * 0.1, 0, 0])
        image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        is_loop, match_idx, matched_pose, sim = pr.detect_loop_closure(t, pose, image)
        if is_loop:
            print(f"Loop closure at t={t:.2f}s, matched frame {match_idx} (sim={sim:.3f})")
