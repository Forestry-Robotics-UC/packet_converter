#!/usr/bin/env python3
import argparse
import sys
import math
import numpy as np
from tqdm import tqdm

# ROS 2 Imports
try:
    import rclpy
    from rclpy.serialization import serialize_message, deserialize_message
    from rosbag2_py import (SequentialReader, StorageOptions, ConverterOptions, 
                            SequentialWriter, TopicMetadata)
    from sensor_msgs.msg import PointCloud2, Imu
    from std_msgs.msg import String, Header
    from sensor_msgs_py import point_cloud2 as pc2
except ImportError as e:
    print(f"Error: Missing ROS 2 dependencies: {e}")
    sys.exit(1)

# Ouster SDK Imports
try:
    from ouster.sdk.core import SensorInfo, XYZLut, LidarScan, ScanBatcher, PacketFormat
    from ouster.sdk._bindings.client import LidarPacket, ImuPacket
except ImportError as e:
    print(f"Error: Ouster Python SDK not found: {e}")
    sys.exit(1)

class BagConverter:
    def __init__(self, args):
        self.args = args
        self.sensor_info = None
        self.xyzlut = None
        self.packet_format = None
        self.scan_batcher = None
        self.lidar_scan = None
        self.imu_count = 0
        self.scan_count = 0
        self.pbar = None

    def run(self):
        # 1. Setup Reader
        reader = SequentialReader()
        reader.open(
            StorageOptions(uri=self.args.input_bag, storage_id='mcap'),
            ConverterOptions('', '')
        )

        # 2. Setup Writer
        writer = SequentialWriter()
        writer.open(
            StorageOptions(uri=self.args.output_bag, storage_id='mcap'),
            ConverterOptions('', '')
        )

        # 3. Transfer Topic Metadata and Register New Topics
        all_reader_topics = reader.get_all_topics_and_types()
        
        # Map to store QoS profiles for our new topics
        input_qos_map = {t.name: t.offered_qos_profiles for t in all_reader_topics}

        # Copy all existing topics to writer
        for topic_meta in all_reader_topics:
            writer.create_topic(topic_meta)

        # Helper to create TopicMetadata safely for Jazzy
        def create_safe_metadata(name, type_str, qos_profiles):
            # Positional arguments: id, name, type, serialization_format, offered_qos_profiles
            return TopicMetadata(
                0, # id (ignored by writer)
                name,
                type_str,
                'cdr',
                qos_profiles if qos_profiles else []
            )

        # Register PointCloud2 topic
        pc_qos = input_qos_map.get(self.args.lidar_packets_topic, [])
        writer.create_topic(create_safe_metadata(self.args.points_topic, 'sensor_msgs/msg/PointCloud2', pc_qos))

        # Register IMU topic
        imu_qos = input_qos_map.get(self.args.imu_packets_topic, [])
        writer.create_topic(create_safe_metadata(self.args.imu_topic, 'sensor_msgs/msg/Imu', imu_qos))
        
        print(f"Processing bag: {self.args.input_bag}")
        self.pbar = tqdm(unit="pkts", desc="Processing", dynamic_ncols=True)

        while reader.has_next():
            topic, data, tstamp = reader.read_next()
            
            # Write original data
            writer.write(topic, data, tstamp)

            if topic == self.args.metadata_topic:
                self.handle_metadata(data)
            elif topic == self.args.imu_packets_topic and self.sensor_info:
                if self.process_imu(data, tstamp, writer):
                    self.update_progress()
            elif topic == self.args.lidar_packets_topic and self.sensor_info:
                if self.process_lidar(data, tstamp, writer):
                    self.update_progress()

        self.pbar.close()
        writer.close()
        print(f"\nFinished. Scans: {self.scan_count}, IMU: {self.imu_count}")

    def handle_metadata(self, data):
        try:
            meta_msg = deserialize_message(data, String)
            self.sensor_info = SensorInfo(meta_msg.data)
            self.xyzlut = XYZLut(self.sensor_info)
            self.packet_format = PacketFormat(self.sensor_info)
            self.scan_batcher = ScanBatcher(self.sensor_info)
            
            w = self.sensor_info.format.columns_per_frame
            h = self.sensor_info.format.pixels_per_column
            self.lidar_scan = LidarScan(h, w)
            self.pbar.write(f"[*] Ouster Metadata Initialized: {self.sensor_info.prod_line}")
        except Exception as e:
            self.pbar.write(f"[!] Metadata Error: {e}")

    def get_raw_buffer(self, data):
        try:
            from ouster_sensor_msgs.msg import PacketMsg
            pkt_msg = deserialize_message(data, PacketMsg)
            return bytes(pkt_msg.buf)
        except Exception:
            return bytes(data)

    def update_progress(self):
        self.pbar.set_postfix(scans=self.scan_count, imu=self.imu_count)
        self.pbar.update(1)

    def process_imu(self, data, tstamp, writer):
        try:
            raw_buf = self.get_raw_buffer(data)
            imu_pkt = ImuPacket(self.packet_format.imu_packet_size)
            np.frombuffer(imu_pkt.buf, dtype=np.uint8)[:] = np.frombuffer(raw_buf, dtype=np.uint8)
            
            imu_msg = Imu()
            imu_msg.header.stamp = rclpy.time.Time(nanoseconds=tstamp).to_msg()
            imu_msg.header.frame_id = self.args.imu_frame_id
            
            imu_msg.linear_acceleration.x = self.packet_format.imu_la_x(imu_pkt.buf) * 9.80665
            imu_msg.linear_acceleration.y = self.packet_format.imu_la_y(imu_pkt.buf) * 9.80665
            imu_msg.linear_acceleration.z = self.packet_format.imu_la_z(imu_pkt.buf) * 9.80665
            imu_msg.angular_velocity.x = math.radians(self.packet_format.imu_av_x(imu_pkt.buf))
            imu_msg.angular_velocity.y = math.radians(self.packet_format.imu_av_y(imu_pkt.buf))
            imu_msg.angular_velocity.z = math.radians(self.packet_format.imu_av_z(imu_pkt.buf))

            writer.write(self.args.imu_topic, serialize_message(imu_msg), tstamp)
            self.imu_count += 1
            return True
        except Exception:
            return False

    def process_lidar(self, data, tstamp, writer):
        try:
            raw_buf = self.get_raw_buffer(data)
            lidar_pkt = LidarPacket(self.packet_format.lidar_packet_size)
            dst = np.frombuffer(lidar_pkt.buf, dtype=np.uint8)
            src = np.frombuffer(raw_buf, dtype=np.uint8)
            dst[:len(src)] = src[:len(dst)]
            
            if self.scan_batcher(lidar_pkt, self.lidar_scan):
                xyz = self.xyzlut(self.lidar_scan)
                points = xyz.reshape(-1, 3)
                mask = np.any(points != 0, axis=1)
                valid_points = points[mask]

                if valid_points.size > 0:
                    header = Header()
                    header.stamp = rclpy.time.Time(nanoseconds=tstamp).to_msg()
                    header.frame_id = self.args.lidar_frame_id
                    
                    pc_msg = pc2.create_cloud_xyz32(header, valid_points)
                    writer.write(self.args.points_topic, serialize_message(pc_msg), tstamp)
                    self.scan_count += 1
                return True
            return False
        except Exception as e:
            return False

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Ouster Bag Converter - Corrected Metadata')
    parser.add_argument('--input-bag', '-i', required=True)
    parser.add_argument('--output-bag', '-o', required=True)
    parser.add_argument('--metadata-topic', default='/ouster/metadata')
    parser.add_argument('--lidar-packets-topic', default='/ouster/lidar_packets')
    parser.add_argument('--imu-packets-topic', default='/ouster/imu_packets')
    parser.add_argument('--points-topic', default='/ouster/points')
    parser.add_argument('--imu-topic', default='/ouster/imu')
    parser.add_argument('--lidar-frame-id', default='os_lidar')
    parser.add_argument('--imu-frame-id', default='os_imu')
    
    args = parser.parse_args()
    rclpy.init()
    try:
        BagConverter(args).run()
    finally:
        rclpy.shutdown()