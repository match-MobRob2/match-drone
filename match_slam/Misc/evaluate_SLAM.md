Here’s what’s typically used to evaluate SLAM with datasets like Newer College:

1. Trajectory evaluation

ATE (Absolute Trajectory Error)
→ Compares the estimated poses vs. ground-truth poses after alignment.

RPE (Relative Pose Error)
→ Checks local drift by comparing relative motion over fixed time/length windows.

Tools:

evo (most common, Python, ROS-ready):
pip install evo
evo_ape tum gt.txt est.txt
evo_rpe tum gt.txt est.txt

Outputs error stats + plots.

2. Map evaluation

Cloud-to-cloud distance (C2C):
→ Measure average / max distance between SLAM map and ground-truth map points.

Tool: CloudCompare (GUI) or Open3D / PCL (scripted).

ICP alignment RMSE:
→ Align SLAM map to ground-truth map with ICP, check RMSE.

Tool: Open3D, PCL, or CloudCompare.

Occupancy/grid-based IoU (if you voxelize):
→ Intersection-over-Union between SLAM occupancy grid and ground-truth voxel grid.


3. Visualization sanity check

Overlay /lio_map (FAST-LIO2 output) and /gt_cloud (ground truth PCD) in RViz2.

Quick visual drift/scale check.

👉 Summary:

Trajectory → evo (ATE/RPE).

Map → Cloud-to-cloud or ICP RMSE.

Always visualize in RViz2.

Do you want me to prepare a step-by-step workflow with evo + CloudCompare so you can run the metrics right after replaying a sequence?