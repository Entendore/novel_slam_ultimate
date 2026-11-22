"""
Long-term Multi-session SLAM Controller
Session 2: Re-localize in existing map from Session 1
"""
from controller import Supervisor
import math
import numpy as np
import pickle
import os

class LongTermSLAM:
    def __init__(self, session_id=2, map_file="results/saved_map.pkl"):
        self.session_id = session_id
        self.map_file = map_file
        self.map_loaded = False
        self.keyframes = []
        self.descriptors = []
        self.reference_map = None
        self.relocalized = False
        
    def save_map(self):
        map_data = {
            'keyframes': self.keyframes,
            'descriptors': self.descriptors,
        }
        with open(self.map_file, 'wb') as f:
            pickle.dump(map_data, f)
        print(f"✅ Map saved to {self.map_file}")
    
    def load_map(self):
        if os.path.exists(self.map_file):
            with open(self.map_file, 'rb') as f:
                self.reference_map = pickle.load(f)
            self.map_loaded = True
            print(f"✅ Loaded map: {len(self.reference_map['keyframes'])} keyframes")
            return True
        return False
    
    def relocalize(self, current_descriptor):
        if not self.map_loaded:
            return None
        best_match = None
        best_score = 0
        for i, ref_desc in enumerate(self.reference_map['descriptors']):
            score = np.dot(current_descriptor, ref_desc) / (
                np.linalg.norm(current_descriptor) * np.linalg.norm(ref_desc) + 1e-8
            )
            if score > best_score:
                best_score = score
                best_match = i
        if best_score > 0.7:
            ref_pose = self.reference_map['keyframes'][best_match]
            print(f"🔄 Re-localized! Matched keyframe {best_match} (score: {best_score:.3f})")
            self.relocalized = True
            return ref_pose
        return None


robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# Parse session from custom data
session_id = 2
custom_data = robot.getCustomData()
for item in custom_data.split(";"):
    if "session:" in item:
        session_id = int(item.split(":")[1])

slam = LongTermSLAM(session_id=session_id)
if session_id == 2:
    slam.load_map()

# Devices
lidar = robot.getDevice('lidar')
lidar.enable(timestep)
imu = robot.getDevice('imu')
imu.enable(timestep)
camera = robot.getDevice('camera')
camera.enable(timestep)
recognition = robot.getDevice('recognition')
recognition.enable(timestep)
gps = robot.getDevice('gps')
gps.enable(timestep)

left_motor = robot.getDevice('left_wheel')
right_motor = robot.getDevice('right_wheel')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_sensor = robot.getDevice('left_sensor')
right_sensor = robot.getDevice('right_sensor')
left_sensor.enable(timestep)
right_sensor.enable(timestep)

trans_field = robot.getSelf().getField("translation")

x, y, theta = 0.0, 0.0, 0.0
prev_l, prev_r = 0.0, 0.0

print(f"Starting Session {session_id}...")

while robot.step(timestep) != -1:
    t = robot.getTime()
    
    lidar_ranges = lidar.getRangeImage()
    camera_image = camera.getImage()
    
    # Extract descriptor (simplified - replace with real feature extractor)
    descriptor = np.random.randn(128)
    
    # Odometry
    dl = (left_sensor.getValue() - prev_l) * 0.1
    dr = (right_sensor.getValue() - prev_r) * 0.1
    prev_l, prev_r = left_sensor.getValue(), right_sensor.getValue()
    
    d_center = (dl + dr) / 2.0
    d_theta = (dr - dl) / 0.5
    x += d_center * math.cos(theta + d_theta / 2.0)
    y += d_center * math.sin(theta + d_theta / 2.0)
    theta += d_theta
    theta = math.atan2(math.sin(theta), math.cos(theta))
    
    if session_id == 1:
        slam.keyframes.append([x, y, theta])
        slam.descriptors.append(descriptor)
    elif session_id == 2:
        if not slam.relocalized:
            ref_pose = slam.relocalize(descriptor)
            if ref_pose is not None:
                x, y, theta = ref_pose
    
    # Drive
    speed = 2.0
    left_motor.setVelocity(speed * 0.8)
    right_motor.setVelocity(speed * 0.4)
    
    # Save map at end
    if t > 60.0:
        slam.save_map()
        break

print(f"Session {session_id} complete.")
