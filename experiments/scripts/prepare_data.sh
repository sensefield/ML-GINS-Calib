#!/usr/bin/env bash
# =============================================================================
# Convert a rosbag (.mcap) -> ML-GINS-Calib input data.
#
# Output layout (created under <output_dir>):
#   <output_dir>/gins_sync/gins_sync.txt   GNSS/INS poses, one per frame
#   <output_dir>/top/<timestamp>.pcd       synchronized top-LiDAR clouds
#   <output_dir>/front/<timestamp>.pcd
#   <output_dir>/back/<timestamp>.pcd
#
# Requirements: ROS 2 Python env (rosbag2_py, rclpy, sensor_msgs, geometry_msgs).
# Run after `source /opt/ros/<distro>/setup.bash`.
#
# Usage:
#   ./experiments/scripts/prepare_data.sh <bag.mcap> <output_dir>
#
# Notes:
#   - frame_step decimates the reference LiDAR (keep every N-th frame).
#   - Verify the topic names against your bag (`ros2 bag info <bag>`).
# =============================================================================
set -euo pipefail

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$REPO"

BAG_PATH="${1:?usage: prepare_data.sh <bag.mcap> <output_dir>}"
OUTPUT_DIR="${2:?usage: prepare_data.sh <bag.mcap> <output_dir>}"

python3 prepare_data.py \
    --bag_path "$BAG_PATH" \
    --output_dir "$OUTPUT_DIR" \
    --gnss_topic /sensing/gnss/pose \
    --frame_step 5 \
    --max_time_diff 0.05 \
    --lidar_topics \
        /sensing/lidar/top/pointcloud_raw_ex:top \
        /sensing/lidar/front/pointcloud_raw_ex:front \
        /sensing/lidar/back/pointcloud_raw_ex:back
