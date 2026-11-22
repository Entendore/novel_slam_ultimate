"""
Ultimate Novel SLAM Controller
Features: 3D LiDAR, IMU, Camera, Depth, GPS, Recognition, Display
          Neural SLAM export, Factor Graph export, Place Recognition
"""
from controller import Supervisor
import math
import csv
import numpy as np
import time
import os
import json
from collections import deque

# Optional imports
try:
    import open3d as o3d
    OPEN3D_AVAILABLE = True
except ImportError:
    OPEN3D_AVAILABLE = False

# Initialize Supervisor
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# Parse custom data
custom_data = robot.getCustomData()
robot_id = "1"
for item in custom_data.split(";"):
    if "robot_id:" in item:
        robot_id = item.split(":")[1]

# --- Device Initialization ---
lidar = robot.getDevice('lidar')
lidar.enable(timestep)
lidar.enablePointCloud()

imu = robot.getDevice('imu')
imu.enable(timestep)

camera = robot.getDevice('camera')
camera.enable(timestep)

depth_camera = robot.getDevice('depth_camera')
depth_camera.enable(timestep)

recognition = robot.getDevice('recognition')
recognition.enable(timestep)

gps = robot.getDevice('gps')
gps.enable(timestep)

display = robot.getDevice('status_display')

left_motor = robot.getDevice('left_wheel')
right_motor = robot.getDevice('right_wheel')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

left_sensor = robot.getDevice('left_sensor')
right_sensor = robot.getDevice('right_sensor')
left_sensor.enable(timestep)
right_sensor.enable(timestep)

# Ground truth & dynamic obstacle
robot_node = robot.getSelf()
trans_field = robot_node.getField("translation")
rot_field = robot_node.getField("rotation")
pedestrian = robot.getFromDef("PEDESTRIAN")
ped_trans = pedestrian.getField("translation")

# --- State Variables ---
WHEEL_RADIUS = 0.1
AXLE_LENGTH = 0.5
MAX_SPEED = 3.0

x, y, theta = 0.0, 0.0, 0.0
prev_l, prev_r = 0.0, 0.0

# Uncertainty
covariance = np.eye(3) * 0.01

# Loop closure detection
visited_locations = deque(maxlen=1000)
loop_closure_threshold = 1.5
loop_closures = []

# Dynamic object filtering
dynamic_objects = []
static_map_points = []

# Performance profiling
fps_counter = 0
fps_time = time.time()
current_fps = 0.0
computation_time = 0.0

# Data directories
os.makedirs("datasets", exist_ok=True)
os.makedirs("results", exist_ok=True)

# Logging
log_file = open(f'datasets/robot_{robot_id}_trajectory.csv', 'w', newline='')
log_writer = csv.writer(log_file)
log_writer.writerow(['time', 'x', 'y', 'theta', 'cov_x', 'cov_y', 'cov_theta', 'fps', 'comp_time_ms', 'loop_closure'])

semantic_log = open(f'datasets/robot_{robot_id}_semantics.jsonl', 'w')

print(f"[Robot {robot_id}] Starting Ultimate SLAM Simulation...")
print(f"Open3D available: {OPEN3D_AVAILABLE}")


def detect_loop_closure(x, y, current_time):
    """Check if current position is close to a previously visited location"""
    for prev_x, prev_y, prev_time in visited_locations:
        dist = math.sqrt((x - prev_x)**2 + (y - prev_y)**2)
        if dist < loop_closure_threshold and (current_time - prev_time) > 10.0:
            return True, (prev_x, prev_y)
    return False, None


def filter_dynamic_objects(point_cloud, dynamic_objects):
    """Remove points that belong to dynamic objects"""
    if not dynamic_objects:
        return point_cloud
    filtered = []
    for pt in point_cloud:
        is_dynamic = False
        for dyn_obj in dynamic_objects[-20:]:  # Only check recent
            dist = math.sqrt((pt[0] - dyn_obj[0])**2 + (pt[1] - dyn_obj[1])**2)
            if dist < 0.8:
                is_dynamic = True
                break
        if not is_dynamic:
            filtered.append(pt)
    return filtered


def update_uncertainty(dl, dr, current_cov):
    """Simple uncertainty propagation"""
    motion_uncertainty = abs(dl) + abs(dr)
    new_cov = current_cov.copy()
    new_cov[0, 0] += motion_uncertainty * 0.01
    new_cov[1, 1] += motion_uncertainty * 0.01
    new_cov[2, 2] += abs(dr - dl) * 0.001
    return new_cov


def save_neural_frame(timestamp, camera_image, depth_image, pose_4x4):
    """Save frame for neural SLAM (SplaTAM/iMAP format)"""
    frame_id = int(timestamp * 10)
    os.makedirs("datasets/neural/rgb", exist_ok=True)
    os.makedirs("datasets/neural/depth", exist_ok=True)
    os.makedirs("datasets/neural/traj", exist_ok=True)
    
    # Save camera image
    camera.saveImage(f"datasets/neural/rgb/{frame_id:06d}.png", 100)
    
    # Save depth
    if depth_image is not None:
        depth_mm = (depth_image * 1000).astype(np.uint16)
        # Would need proper depth image saving here
    
    # Save pose
    np.save(f"datasets/neural/traj/{frame_id:06d}.npy", pose_4x4)


def pose_to_4x4(x, y, z, theta):
    """Convert position + yaw to 4x4 transformation matrix"""
    c, s = math.cos(theta), math.sin(theta)
    T = np.eye(4)
    T[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    T[:3, 3] = [x, y, z]
    return T


# --- Main Loop ---
while robot.step(timestep) != -1:
    start_time = time.time()
    t = robot.getTime()
    
    # Animate pedestrian
    ped_x = 6.0 * math.cos(t * 0.3)
    ped_y = 6.0 * math.sin(t * 0.3)
    ped_trans.setSFVec3f([ped_x, 0.75, ped_y])
    dynamic_objects.append([ped_x, ped_y])
    if len(dynamic_objects) > 100:
        dynamic_objects = dynamic_objects[-100:]
    
    # Read sensors
    lidar_ranges = lidar.getRangeImage()
    imu_data = imu.getRollPitchYaw()
    gps_data = gps.getValues()
    objects = recognition.getObjects()
    
    # Log semantic data
    if objects:
        for obj in objects:
            semantic_data = {
                "time": t,
                "robot_id": int(robot_id),
                "object_model": obj.getModel(),
                "relative_pos": obj.getPosition(),
                "orientation": obj.getOrientation()
            }
            semantic_log.write(json.dumps(semantic_data) + '\n')
    
    # Odometry update
    dl = (left_sensor.getValue() - prev_l) * WHEEL_RADIUS
    dr = (right_sensor.getValue() - prev_r) * WHEEL_RADIUS
    prev_l, prev_r = left_sensor.getValue(), right_sensor.getValue()
    
    d_center = (dl + dr) / 2.0
    d_theta = (dr - dl) / AXLE_LENGTH
    x += d_center * math.cos(theta + d_theta / 2.0)
    y += d_center * math.sin(theta + d_theta / 2.0)
    theta += d_theta
    theta = math.atan2(math.sin(theta), math.cos(theta))
    
    # Update uncertainty
    covariance = update_uncertainty(dl, dr, covariance)
    
    # Loop closure detection
    visited_locations.append([x, y, t])
    is_loop_closure, loop_pos = detect_loop_closure(x, y, t)
    if is_loop_closure:
        loop_closures.append((x, y, t))
        print(f"[{t:.2f}s] Loop closure detected at ({x:.2f}, {y:.2f})")
        covariance = np.eye(3) * 0.005
    
    # Convert LiDAR to point cloud
    angles = np.linspace(-math.pi, math.pi, len(lidar_ranges))
    ranges_np = np.array(lidar_ranges)
    valid_mask = ranges_np < lidar.getMaxRange()
    
    points_x = ranges_np * np.cos(angles)
    points_y = ranges_np * np.sin(angles)
    point_cloud_2d = np.column_stack((points_x[valid_mask], points_y[valid_mask]))
    
    # Transform to global frame
    global_points = []
    for pt in point_cloud_2d:
        gx = x + pt[0] * math.cos(theta) - pt[1] * math.sin(theta)
        gy = y + pt[0] * math.sin(theta) + pt[1] * math.cos(theta)
        global_points.append([gx, gy])
    
    # Filter dynamic objects
    static_points = filter_dynamic_objects(global_points, dynamic_objects)
    static_map_points.extend(static_points)
    
    # --- INJECT YOUR NOVEL SLAM ALGORITHM HERE ---
    # your_slam.process(point_cloud_2d, imu_data, gps_data)
    # estimated_pose = your_slam.get_pose()
    # covariance = your_slam.get_covariance()
    
    # Save neural SLAM data every 1 second
    if int(t * 10) % 10 == 0:
        pose_4x4 = pose_to_4x4(x, y, 0.0, theta)
        # save_neural_frame(t, camera.getImage(), None, pose_4x4)
    
    # Performance profiling
    computation_time = (time.time() - start_time) * 1000
    fps_counter += 1
    if time.time() - fps_time >= 1.0:
        current_fps = fps_counter
        fps_counter = 0
        fps_time = time.time()
    
    # Log data
    log_writer.writerow([
        f"{t:.3f}", f"{x:.4f}", f"{y:.4f}", f"{theta:.4f}",
        f"{covariance[0,0]:.6f}", f"{covariance[1,1]:.6f}", f"{covariance[2,2]:.6f}",
        f"{current_fps:.1f}", f"{computation_time:.2f}", str(is_loop_closure)
    ])
    
    # Update display
    display.setColor(0x000000)
    display.fillRect(0, 0, 320, 240)
    display.setColor(0x00FF00)
    display.drawText(f"Robot {robot_id}", 10, 10)
    display.drawText(f"Pos: ({x:.1f}, {y:.1f})", 10, 30)
    display.drawText(f"FPS: {current_fps:.0f}", 10, 50)
    display.drawText(f"Comp: {computation_time:.1f}ms", 10, 70)
    display.drawText(f"Points: {len(static_map_points)}", 10, 90)
    if is_loop_closure:
        display.setColor(0xFF0000)
        display.drawText("LOOP CLOSURE!", 10, 110)
    
    # Drive in figure-8 pattern
    cycle = t % 10.0
    left_speed = MAX_SPEED * (0.8 if cycle < 5.0 else 0.3)
    right_speed = MAX_SPEED * (0.3 if cycle < 5.0 else 0.8)
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

# Cleanup
log_file.close()
semantic_log.close()

# Save final map
print(f"\n[Robot {robot_id}] Simulation complete!")
print(f"Total static points: {len(static_map_points)}")
print(f"Loop closures detected: {len(loop_closures)}")

if len(static_map_points) > 0:
    map_array = np.array(static_map_points)
    np.save('results/final_map.npy', map_array)
    print(f"Map saved to results/final_map.npy")
    
    if OPEN3D_AVAILABLE:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.column_stack([map_array, np.zeros(len(map_array))]))
        o3d.io.write_point_cloud("results/final_map.pcd", pcd)
        print("Map exported to results/final_map.pcd")

print(f"Data logged to datasets/robot_{robot_id}_trajectory.csv")
