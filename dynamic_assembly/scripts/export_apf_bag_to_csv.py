#!/usr/bin/env python3
"""Export APF/MPC local avoidance topics from a rosbag2 bag to CSV files."""

import argparse
import csv
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


DEFAULT_TOPICS = {
    "/strategic_goal_path": "strategic_goal_path.csv",
    "/strategic_goal_path_marker": "strategic_goal_path_marker.csv",
    "/predicted_target_trajectory": "predicted_target_trajectory.csv",
    "/estimated_object_state": "estimated_object_state.csv",
    "/end_effector_path": "end_effector_path.csv",
}



def detect_storage_id(bag_path):
    metadata_path = bag_path / 'metadata.yaml'
    if not metadata_path.is_file():
        return None

    for line in metadata_path.read_text(encoding='utf-8').splitlines():
        stripped = line.strip()
        if stripped.startswith('storage_identifier:'):
            value = stripped.split(':', 1)[1].strip()
            return value.strip(chr(39) + chr(34))
    return None
def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def write_rows(path, fieldnames, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def path_rows(msg, bag_time, msg_index):
    rows = []
    for point_index, pose_stamped in enumerate(msg.poses):
        position = pose_stamped.pose.position
        rows.append({
            "bag_time": bag_time,
            "msg_index": msg_index,
            "path_stamp": stamp_to_sec(msg.header.stamp),
            "point_stamp": stamp_to_sec(pose_stamped.header.stamp),
            "point_index": point_index,
            "frame_id": pose_stamped.header.frame_id or msg.header.frame_id,
            "x": position.x,
            "y": position.y,
            "z": position.z,
        })
    return rows


def odometry_row(msg, bag_time, msg_index):
    position = msg.pose.pose.position
    twist = msg.twist.twist.linear
    return {
        "bag_time": bag_time,
        "msg_index": msg_index,
        "stamp": stamp_to_sec(msg.header.stamp),
        "frame_id": msg.header.frame_id,
        "x": position.x,
        "y": position.y,
        "z": position.z,
        "vx": twist.x,
        "vy": twist.y,
        "vz": twist.z,
    }


def marker_rows(msg, bag_time, msg_index):
    rows = []
    for point_index, point in enumerate(msg.points):
        rows.append({
            "bag_time": bag_time,
            "msg_index": msg_index,
            "stamp": stamp_to_sec(msg.header.stamp),
            "point_index": point_index,
            "frame_id": msg.header.frame_id,
            "namespace": msg.ns,
            "marker_id": msg.id,
            "type": msg.type,
            "x": point.x,
            "y": point.y,
            "z": point.z,
        })
    return rows


def parse_obstacle(text):
    values = [float(part.strip()) for part in text.split(",")]
    if len(values) not in (3, 4, 5):
        raise argparse.ArgumentTypeError(
            "obstacle must be x,y,r or x,y,z,r or x,y,z,r,influence_radius")
    if len(values) == 3:
        return {
            "x": values[0],
            "y": values[1],
            "z": 0.0,
            "radius": values[2],
            "influence_radius": float("nan"),
        }
    if len(values) == 4:
        return {
            "x": values[0],
            "y": values[1],
            "z": values[2],
            "radius": values[3],
            "influence_radius": float("nan"),
        }
    return {
        "x": values[0],
        "y": values[1],
        "z": values[2],
        "radius": values[3],
        "influence_radius": values[4],
    }


def export_bag(args):
    reader = rosbag2_py.SequentialReader()
    storage_id = args.storage_id
    if storage_id == 'auto':
        storage_id = detect_storage_id(args.bag) or 'sqlite3'

    storage_options = rosbag2_py.StorageOptions(uri=str(args.bag), storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)

    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    message_types = {
        topic: get_message(topic_type)
        for topic, topic_type in topic_types.items()
        if topic in DEFAULT_TOPICS
    }

    rows_by_topic = {topic: [] for topic in DEFAULT_TOPICS}
    message_counts = {topic: 0 for topic in DEFAULT_TOPICS}

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic not in message_types:
            continue

        msg = deserialize_message(data, message_types[topic])
        bag_time = float(timestamp) * 1.0e-9
        msg_index = message_counts[topic]
        message_counts[topic] += 1

        if topic in ("/strategic_goal_path", "/predicted_target_trajectory", "/end_effector_path"):
            rows_by_topic[topic].extend(path_rows(msg, bag_time, msg_index))
        elif topic == "/estimated_object_state":
            rows_by_topic[topic].append(odometry_row(msg, bag_time, msg_index))
        elif topic == "/strategic_goal_path_marker":
            rows_by_topic[topic].extend(marker_rows(msg, bag_time, msg_index))

    output_dir = args.output_dir
    path_fields = [
        "bag_time",
        "msg_index",
        "path_stamp",
        "point_stamp",
        "point_index",
        "frame_id",
        "x",
        "y",
        "z",
    ]
    odom_fields = ["bag_time", "msg_index", "stamp", "frame_id", "x", "y", "z", "vx", "vy", "vz"]
    marker_fields = [
        "bag_time",
        "msg_index",
        "stamp",
        "point_index",
        "frame_id",
        "namespace",
        "marker_id",
        "type",
        "x",
        "y",
        "z",
    ]

    for topic, filename in DEFAULT_TOPICS.items():
        if topic in ("/strategic_goal_path", "/predicted_target_trajectory", "/end_effector_path"):
            fields = path_fields
        elif topic == "/estimated_object_state":
            fields = odom_fields
        else:
            fields = marker_fields
        write_rows(output_dir / filename, fields, rows_by_topic[topic])

    if args.obstacle:
        write_rows(
            output_dir / "obstacles.csv",
            ["x", "y", "z", "radius", "influence_radius"],
            args.obstacle,
        )

    print(f'Opened rosbag with storage id: {storage_id}')
    print(f"Exported APF local avoidance CSV files to: {output_dir}")
    for topic, filename in DEFAULT_TOPICS.items():
        print(f"  {filename}: {len(rows_by_topic[topic])} rows")
    if args.obstacle:
        print(f"  obstacles.csv: {len(args.obstacle)} rows")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="Path to the rosbag2 directory.")
    parser.add_argument(
        "-o",
        "--output-dir",
        type=Path,
        default=Path("apf_local_avoidance_csv"),
        help="Directory for generated CSV files.",
    )
    parser.add_argument(
        "--storage-id",
        default='auto',
        help='rosbag2 storage id: auto, sqlite3, or mcap.',
    )
    parser.add_argument(
        "--obstacle",
        type=parse_obstacle,
        action="append",
        help="Obstacle as x,y,r or x,y,z,r or x,y,z,r,influence_radius. Repeat for multiple obstacles.",
    )
    args = parser.parse_args()
    export_bag(args)


if __name__ == "__main__":
    main()
