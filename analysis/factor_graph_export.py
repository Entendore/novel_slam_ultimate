"""
Export SLAM factor graph in g2o format for visualization
"""
import numpy as np

class G2OExporter:
    def __init__(self, output_file="results/graph.g2o"):
        self.output_file = output_file
        self.vertices = []
        self.edges = []
        self.vertex_id = 0
    
    def add_pose_vertex(self, x, y, theta):
        self.vertices.append(f"VERTEX_SE2 {self.vertex_id} {x} {y} {theta}")
        vid = self.vertex_id
        self.vertex_id += 1
        return vid
    
    def add_odometry_edge(self, from_id, to_id, dx, dy, dtheta, information):
        info = f"{information[0,0]:.6f} {information[0,1]:.6f} {information[0,2]:.6f} {information[1,1]:.6f} {information[1,2]:.6f} {information[2,2]:.6f}"
        self.edges.append(f"EDGE_SE2 {from_id} {to_id} {dx} {dy} {dtheta} {info}")
    
    def add_loop_closure_edge(self, from_id, to_id, dx, dy, dtheta, information):
        info = f"{information[0,0]:.6f} {information[0,1]:.6f} {information[0,2]:.6f} {information[1,1]:.6f} {information[1,2]:.6f} {information[2,2]:.6f}"
        self.edges.append(f"EDGE_SE2 {from_id} {to_id} {dx} {dy} {dtheta} {info}")
    
    def add_prior_edge(self, vertex_id, x, y, theta, information):
        info = f"{information[0,0]:.6f} {information[0,1]:.6f} {information[0,2]:.6f} {information[1,1]:.6f} {information[1,2]:.6f} {information[2,2]:.6f}"
        self.edges.append(f"EDGE_SE2_PRIOR {vertex_id} {x} {y} {theta} {info}")
    
    def save(self):
        with open(self.output_file, 'w') as f:
            f.write("# g2o factor graph from Novel SLAM\n")
            f.write(f"# Vertices: {len(self.vertices)}, Edges: {len(self.edges)}\n")
            f.write("FIX 0\n")
            for v in self.vertices:
                f.write(v + "\n")
            for e in self.edges:
                f.write(e + "\n")
        print(f"✅ Factor graph exported to {self.output_file}")


def build_example_graph():
    exporter = G2OExporter("results/example_graph.g2o")
    odom_info = np.diag([100, 100, 500])
    loop_info = np.diag([1000, 1000, 2000])
    prior_info = np.diag([1e6, 1e6, 1e6])
    
    prev_id = None
    first_id = None
    for i in range(10):
        theta = 2 * np.pi * i / 10
        x = 5 * np.cos(theta)
        y = 5 * np.sin(theta)
        vid = exporter.add_pose_vertex(x, y, theta)
        if i == 0:
            first_id = vid
            exporter.add_prior_edge(vid, 0, 0, 0, prior_info)
        else:
            dx = x - prev_x
            dy = y - prev_y
            dtheta = theta - prev_theta
            exporter.add_odometry_edge(prev_id, vid, dx, dy, dtheta, odom_info)
            if i == 9:
                exporter.add_loop_closure_edge(vid, first_id, -x, -y, -theta, loop_info)
        prev_id = vid
        prev_x, prev_y, prev_theta = x, y, theta
    exporter.save()

if __name__ == "__main__":
    build_example_graph()
