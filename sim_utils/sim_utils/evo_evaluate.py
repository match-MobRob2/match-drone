import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import subprocess, os, datetime

class OdomRecorder(Node):
    def __init__(self, results_dir):
        super().__init__("evo_evaluate")
        self.results_dir = results_dir
        os.makedirs(self.results_dir, exist_ok=True)

        self.gt_file = open(os.path.join(self.results_dir, "gt.txt"), "w")
        self.est_file = open(os.path.join(self.results_dir, "est.txt"), "w")

        self.create_subscription(Odometry, "/odom", self.gt_callback, 10)
        self.create_subscription(Odometry, "/Odometry", self.est_callback, 10)
        self.get_logger().info(f"Recording /odom -> {self.results_dir}/gt.txt "
                               f"and /Odometry -> {self.results_dir}/est.txt")

    def gt_callback(self, msg):
        self.write_tum(self.gt_file, msg)

    def est_callback(self, msg):
        self.write_tum(self.est_file, msg)

    def write_tum(self, f, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        line = f"{t:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} {q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}"
        parts = line.strip().split()
        assert len(parts) == 8, f"Invalid TUM line: {line}"
        f.write(" ".join(parts) + "\n")

def run_evo_and_log(cmd, log_file):
    """Run evo command, capture stdout, and append to log file."""
    result = subprocess.run(cmd, capture_output=True, text=True)
    with open(log_file, "a") as f:
        f.write("\n>>> " + " ".join(cmd) + "\n")
        f.write(result.stdout + "\n")
    if result.returncode != 0:
        print(f"Error running {' '.join(cmd)}:\n{result.stderr}")

def main():
    rclpy.init()

    # Create timestamped results directory
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    results_dir = os.path.join("evo_evaluation", timestamp)
    os.makedirs(results_dir, exist_ok=True)

    node = OdomRecorder(results_dir)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping recorder... running EVO evaluation.")
    finally:
        node.gt_file.close()
        node.est_file.close()

        log_file = os.path.join(results_dir, "evo_stats.txt")

        # Absolute Trajectory Error (ATE) - 2D
        run_evo_and_log([
            "evo_ape", "tum",
            os.path.join(results_dir, "gt.txt"),
            os.path.join(results_dir, "est.txt"),
            "-va",
            "--plot", "--plot_mode", "xy",
            "--save_plot", os.path.join(results_dir, "ate_traj_xy.pdf"),
            # "--histogram",
            "--save_results", os.path.join(results_dir, "ate_results.zip")
        ], log_file)

        # ATE 3D
        run_evo_and_log([
            "evo_ape", "tum",
            os.path.join(results_dir, "gt.txt"),
            os.path.join(results_dir, "est.txt"),
            "-va",
            "--plot", "--plot_mode", "xyz",
            "--save_plot", os.path.join(results_dir, "ate_traj_xyz.pdf"),
            "--save_results", os.path.join(results_dir, "ate_results_xyz.zip")
        ], log_file)

        # Relative Pose Error (RPE) - 2D
        run_evo_and_log([
            "evo_rpe", "tum",
            os.path.join(results_dir, "gt.txt"),
            os.path.join(results_dir, "est.txt"),
            "-va",
            "--plot", "--plot_mode", "xyz",
            "--save_plot", os.path.join(results_dir, "rpe_traj_xyz.pdf"),
            # "--histogram", "--boxplot",
            "--save_results", os.path.join(results_dir, "rpe_results.zip")
        ], log_file)

        # RPE 3D
        run_evo_and_log([
            "evo_rpe", "tum",
            os.path.join(results_dir, "gt.txt"),
            os.path.join(results_dir, "est.txt"),
            "-va",
            "--plot", "--plot_mode", "xyz",
            "--save_plot", os.path.join(results_dir, "rpe_traj_xyz.pdf"),
            "--save_results", os.path.join(results_dir, "rpe_results_xyz.zip")
        ], log_file)

        node.destroy_node()
        # rclpy.shutdown()

if __name__ == "__main__":
    main()
