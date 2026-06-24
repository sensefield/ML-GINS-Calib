#!/usr/bin/env python3
"""Convert rosbag (mcap) to ML-GINS-Calib input data format."""

import argparse
import os
import sys

import struct

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2


# ---------------------------------------------------------------------------
# rosbag reading
# ---------------------------------------------------------------------------

def read_messages_by_topic(bag_path: str, topic: str) -> list:
    """Read all messages for a given topic from a rosbag.

    Returns list of (timestamp_ns: int, serialized_data: bytes).
    """
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    messages = []
    while reader.has_next():
        topic_name, data, timestamp_ns = reader.read_next()
        messages.append((timestamp_ns, data))
    return messages


def get_available_topics(bag_path: str) -> dict:
    """Return {topic_name: msg_type} from bag metadata."""
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)
    return {t.name: t.type for t in reader.get_all_topics_and_types()}


# ---------------------------------------------------------------------------
# Message parsing
# ---------------------------------------------------------------------------

def quaternion_to_rotation_matrix(w: float, x: float, y: float, z: float) -> np.ndarray:
    """Convert quaternion (w, x, y, z) to 3x3 rotation matrix."""
    R = np.array([
        [1 - 2*(y*y + z*z),  2*(x*y - w*z),      2*(x*z + w*y)],
        [2*(x*y + w*z),      1 - 2*(x*x + z*z),  2*(y*z - w*x)],
        [2*(x*z - w*y),      2*(y*z + w*x),      1 - 2*(x*x + y*y)],
    ])
    return R


def parse_pose_stamped(timestamp_ns: int, data: bytes) -> tuple:
    """Parse PoseStamped message into (timestamp_sec, 4x4 matrix)."""
    msg = deserialize_message(data, PoseStamped)
    t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    p = msg.pose.position
    q = msg.pose.orientation
    R = quaternion_to_rotation_matrix(q.w, q.x, q.y, q.z)

    T = np.eye(4)
    T[:3, :3] = R
    T[0, 3] = p.x
    T[1, 3] = p.y
    T[2, 3] = p.z
    return (t_sec, T)


_ROS_DTYPE_TO_NUMPY = {
    1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
    5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64,
}


def parse_pointcloud2(timestamp_ns: int, data: bytes) -> tuple:
    """Parse PointCloud2 message into (timestamp_sec, Nx4 float32 xyzi array)."""
    msg = deserialize_message(data, PointCloud2)
    t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    # Build numpy dtype from fields
    dtype_list = []
    for f in msg.fields:
        np_type = _ROS_DTYPE_TO_NUMPY.get(f.datatype, np.uint8)
        dtype_list.append((f.name, np_type))
    # Account for padding at end of point_step
    struct_size = sum(np.dtype(dt).itemsize for _, dt in dtype_list)
    if msg.point_step > struct_size:
        dtype_list.append(("_pad", np.uint8, msg.point_step - struct_size))

    dt = np.dtype(dtype_list)
    points_structured = np.frombuffer(msg.data, dtype=dt)

    has_intensity = 'intensity' in points_structured.dtype.names
    x = points_structured['x'].astype(np.float32)
    y = points_structured['y'].astype(np.float32)
    z = points_structured['z'].astype(np.float32)
    intensity = points_structured['intensity'].astype(np.float32) if has_intensity else np.zeros(len(x), dtype=np.float32)

    xyzi = np.column_stack([x, y, z, intensity])
    return (t_sec, xyzi)


# ---------------------------------------------------------------------------
# Time synchronization
# ---------------------------------------------------------------------------

def find_nearest(timestamps: np.ndarray, target: float) -> tuple:
    """Find nearest timestamp index and time difference."""
    idx = np.searchsorted(timestamps, target)
    # Check neighbors
    if idx == 0:
        return 0, abs(timestamps[0] - target)
    if idx >= len(timestamps):
        return len(timestamps) - 1, abs(timestamps[-1] - target)
    # Compare left and right
    if abs(timestamps[idx - 1] - target) <= abs(timestamps[idx] - target):
        return idx - 1, abs(timestamps[idx - 1] - target)
    return idx, abs(timestamps[idx] - target)


def synchronize(gnss_series: list, lidar_series_dict: dict,
                max_time_diff: float = 0.05,
                frame_step: int = 1) -> dict:
    """Synchronize GNSS/INS and LiDAR series by nearest-neighbor matching.

    Uses the LiDAR with fewest messages as the reference timeline.
    `frame_step` decimates the reference timeline (keeps every N-th frame).
    """
    # Find reference LiDAR (fewest messages)
    ref_name = min(lidar_series_dict, key=lambda k: len(lidar_series_dict[k]))
    ref_series = lidar_series_dict[ref_name][::frame_step]
    ref_timestamps = np.array([t for t, _ in ref_series])

    # Prepare timestamp arrays for other series
    gnss_timestamps = np.array([t for t, _ in gnss_series])
    other_lidar_timestamps = {}
    for name, series in lidar_series_dict.items():
        if name != ref_name:
            other_lidar_timestamps[name] = np.array([t for t, _ in series])

    result = {"timestamps": [], "gnss": []}
    for name in lidar_series_dict:
        result[name] = []

    skipped = 0
    used_gnss_indices = set()
    used_lidar_indices = {name: set() for name in other_lidar_timestamps}

    for i, ref_t in enumerate(ref_timestamps):
        # Match GNSS
        gnss_idx, gnss_diff = find_nearest(gnss_timestamps, ref_t)
        if gnss_diff > max_time_diff:
            skipped += 1
            continue
        # Skip if this GNSS frame is already used (avoid duplicates)
        if gnss_idx in used_gnss_indices:
            skipped += 1
            continue

        # Match other LiDARs
        matched = True
        other_indices = {}
        for name, ts_arr in other_lidar_timestamps.items():
            idx, diff = find_nearest(ts_arr, ref_t)
            if diff > max_time_diff:
                matched = False
                print(f"  Warning: LiDAR '{name}' frame {i} time diff {diff:.4f}s > {max_time_diff}s, skipping")
                break
            if idx in used_lidar_indices[name]:
                matched = False
                break
            other_indices[name] = idx

        if not matched:
            skipped += 1
            continue

        # All matched — collect data
        used_gnss_indices.add(gnss_idx)
        for name, idx in other_indices.items():
            used_lidar_indices[name].add(idx)

        result["timestamps"].append(ref_t)
        result["gnss"].append(gnss_series[gnss_idx][1])
        result[ref_name].append(ref_series[i][1])
        for name, idx in other_indices.items():
            result[name].append(lidar_series_dict[name][idx][1])

    print(f"  Reference LiDAR: '{ref_name}' ({len(ref_series)} messages)")
    print(f"  Synchronized frames: {len(result['timestamps'])}")
    print(f"  Skipped frames: {skipped}")

    return result


# ---------------------------------------------------------------------------
# File output
# ---------------------------------------------------------------------------

def save_gnss_ins(output_dir: str, timestamps: list, matrices: list):
    """Save GNSS/INS poses to gins_sync/gins_sync.txt."""
    gins_dir = os.path.join(output_dir, "gins_sync")
    os.makedirs(gins_dir, exist_ok=True)
    out_path = os.path.join(gins_dir, "gins_sync.txt")

    with open(out_path, "w") as f:
        for t, T in zip(timestamps, matrices):
            # Format: timestamp R00 R01 R02 tx R10 R11 R12 ty R20 R21 R22 tz
            vals = [
                T[0, 0], T[0, 1], T[0, 2], T[0, 3],
                T[1, 0], T[1, 1], T[1, 2], T[1, 3],
                T[2, 0], T[2, 1], T[2, 2], T[2, 3],
            ]
            line = f"{t:.9f} " + " ".join(f"{v:.9f}" for v in vals)
            f.write(line + "\n")

    print(f"  Saved GNSS/INS: {out_path} ({len(timestamps)} poses)")


def save_point_cloud(output_dir: str, lidar_name: str, timestamp: float,
                     xyzi: np.ndarray):
    """Save a single point cloud frame as binary PCD with XYZI fields.

    xyzi: Nx4 float32 array (x, y, z, intensity).
    """
    lidar_dir = os.path.join(output_dir, lidar_name)
    os.makedirs(lidar_dir, exist_ok=True)

    n = len(xyzi)
    filename = f"{timestamp:.9f}.pcd"
    filepath = os.path.join(lidar_dir, filename)

    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )

    with open(filepath, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(xyzi.astype(np.float32).tobytes())


def save_all_point_clouds(output_dir: str, lidar_name: str, timestamps: list,
                          point_clouds: list):
    """Save all point cloud frames for a LiDAR."""
    lidar_dir = os.path.join(output_dir, lidar_name)
    os.makedirs(lidar_dir, exist_ok=True)

    for i, (t, pc) in enumerate(zip(timestamps, point_clouds)):
        save_point_cloud(output_dir, lidar_name, t, pc)
        if (i + 1) % 50 == 0 or (i + 1) == len(timestamps):
            print(f"    {lidar_name}: {i + 1}/{len(timestamps)} saved")

    print(f"  Saved LiDAR '{lidar_name}': {len(timestamps)} frames -> {lidar_dir}/")


# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------

def print_summary(output_dir: str, sync_result: dict, raw_counts: dict):
    """Print processing summary."""
    print("\n=== Summary ===")
    print(f"  Synchronized frames: {len(sync_result['timestamps'])}")
    print(f"  GNSS/INS messages (raw): {raw_counts['gnss']}")
    for name in sync_result:
        if name in ("timestamps", "gnss"):
            continue
        print(f"  LiDAR '{name}' messages (raw): {raw_counts.get(name, '?')}")
    print(f"  Output directory: {output_dir}")


# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------

def parse_lidar_topic_arg(arg: str) -> tuple:
    """Parse 'topic:name' argument into (topic, name)."""
    if ":" not in arg:
        print(f"Error: LiDAR topic must be in 'topic:name' format, got: {arg}")
        sys.exit(1)
    topic, name = arg.rsplit(":", 1)
    return topic, name


def parse_args():
    parser = argparse.ArgumentParser(
        description="Convert rosbag (mcap) to ML-GINS-Calib input data format."
    )
    parser.add_argument("--bag_path", required=True, help="Path to rosbag (mcap) file")
    parser.add_argument("--output_dir", default="./data2", help="Output directory (default: ./data2)")
    parser.add_argument("--gnss_topic", default="/sensing/gnss/pose",
                        help="GNSS/INS pose topic (default: /sensing/gnss/pose)")
    parser.add_argument("--lidar_topics", nargs="+", required=True,
                        help="LiDAR topics in 'topic:name' format (e.g. /sensing/lidar/front/pointcloud_raw_ex:front)")
    parser.add_argument("--max_time_diff", type=float, default=0.05,
                        help="Max time difference for synchronization in seconds (default: 0.05)")
    parser.add_argument("--frame_step", type=int, default=1,
                        help="Reference LiDAR decimation step (keep every N-th frame; default: 1 = no decimation)")
    return parser.parse_args()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    args = parse_args()

    # Validate bag file exists
    if not os.path.exists(args.bag_path):
        print(f"Error: Bag file not found: {args.bag_path}")
        sys.exit(1)

    # Parse lidar topic arguments
    lidar_topics = {}  # {name: topic}
    for arg in args.lidar_topics:
        topic, name = parse_lidar_topic_arg(arg)
        lidar_topics[name] = topic

    # Validate topics exist in bag
    available = get_available_topics(args.bag_path)
    all_requested = [args.gnss_topic] + list(lidar_topics.values())
    for topic in all_requested:
        if topic not in available:
            print(f"Error: Topic '{topic}' not found in bag.")
            print(f"Available topics:")
            for t, ty in sorted(available.items()):
                print(f"  {t} [{ty}]")
            sys.exit(1)

    raw_counts = {}

    # --- Read GNSS/INS ---
    print(f"Reading GNSS/INS topic: {args.gnss_topic}")
    gnss_raw = read_messages_by_topic(args.bag_path, args.gnss_topic)
    gnss_series = [parse_pose_stamped(ts, data) for ts, data in gnss_raw]
    raw_counts["gnss"] = len(gnss_series)
    print(f"  {len(gnss_series)} messages read")

    # --- Read LiDAR topics ---
    lidar_series_dict = {}
    for name, topic in lidar_topics.items():
        print(f"Reading LiDAR topic: {topic} -> '{name}'")
        lidar_raw = read_messages_by_topic(args.bag_path, topic)
        lidar_series = [parse_pointcloud2(ts, data) for ts, data in lidar_raw]
        lidar_series_dict[name] = lidar_series
        raw_counts[name] = len(lidar_series)
        print(f"  {len(lidar_series)} messages read")

    # --- Synchronize ---
    print("\nSynchronizing...")
    sync_result = synchronize(gnss_series, lidar_series_dict, args.max_time_diff,
                              frame_step=args.frame_step)

    if len(sync_result["timestamps"]) == 0:
        print("Error: No synchronized frames. Check topics and time alignment.")
        sys.exit(1)

    # --- Save outputs ---
    print(f"\nSaving to: {args.output_dir}")
    os.makedirs(args.output_dir, exist_ok=True)

    save_gnss_ins(args.output_dir, sync_result["timestamps"], sync_result["gnss"])

    for name in lidar_topics:
        save_all_point_clouds(
            args.output_dir, name,
            sync_result["timestamps"], sync_result[name],
        )

    # --- Summary ---
    print_summary(args.output_dir, sync_result, raw_counts)


if __name__ == "__main__":
    main()
