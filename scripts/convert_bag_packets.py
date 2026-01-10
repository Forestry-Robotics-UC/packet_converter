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
        self.first_packet_timestamp = None  # Track first packet timestamp in batch
    
    @staticmethod
    def parse_size(size_str):
        """
        Parse size string to bytes.
        Examples: '500M', '1G', '100K', '1024' (bytes)
        Returns 0 if size_str is None, empty, or '0'
        """
        if not size_str or size_str == '0':
            return 0
        
        size_str = size_str.strip().upper()
        
        # Parse suffix
        multipliers = {
            'K': 1024,
            'M': 1024 * 1024,
            'G': 1024 * 1024 * 1024,
            'KB': 1024,
            'MB': 1024 * 1024,
            'GB': 1024 * 1024 * 1024,
        }
        
        for suffix, multiplier in multipliers.items():
            if size_str.endswith(suffix):
                try:
                    value = float(size_str[:-len(suffix)])
                    return int(value * multiplier)
                except ValueError:
                    raise ValueError(f"Invalid size format: {size_str}")
        
        # No suffix, assume bytes
        try:
            return int(size_str)
        except ValueError:
            raise ValueError(f"Invalid size format: {size_str}")
    
    @staticmethod
    def parse_duration(duration_str):
        """
        Parse duration string to nanoseconds.
        Examples: '60s', '5m', '1h', '30' (seconds)
        Returns 0 if duration_str is None, empty, or '0'
        """
        if not duration_str or duration_str == '0':
            return 0
        
        duration_str = duration_str.strip().lower()
        
        # Parse suffix
        multipliers = {
            's': 1,
            'm': 60,
            'h': 3600,
            'sec': 1,
            'min': 60,
            'hour': 3600,
        }
        
        for suffix, multiplier in multipliers.items():
            if duration_str.endswith(suffix):
                try:
                    value = float(duration_str[:-len(suffix)])
                    return int(value * multiplier * 1e9)  # Convert to nanoseconds
                except ValueError:
                    raise ValueError(f"Invalid duration format: {duration_str}")
        
        # No suffix, assume seconds
        try:
            return int(float(duration_str) * 1e9)
        except ValueError:
            raise ValueError(f"Invalid duration format: {duration_str}")

    def run(self):
        # 1. Setup Reader
        reader = SequentialReader()
        reader.open(
            StorageOptions(uri=self.args.input_bag, storage_id='mcap'),
            ConverterOptions('', '')
        )

        # 2. Get metadata from input bag for informational purposes
        bag_metadata = reader.get_metadata()
        
        # Duration is a Duration object with nanoseconds attribute
        duration_ns = bag_metadata.duration.nanoseconds if hasattr(bag_metadata.duration, 'nanoseconds') else 0
        duration_sec = duration_ns / 1e9
        
        print(f"Input bag info:")
        print(f"  - Storage ID: {bag_metadata.storage_identifier}")
        print(f"  - Duration: {duration_sec:.2f} seconds")
        print(f"  - Message count: {bag_metadata.message_count}")
        print(f"  - Files: {len(bag_metadata.relative_file_paths)}")

        # 3. Setup Writer with split settings from command line arguments
        writer = SequentialWriter()
        output_storage_options = StorageOptions(
            uri=self.args.output_bag,
            storage_id='mcap'
        )

        # Parse split settings
        max_bagfile_size = self.parse_size(self.args.max_bagfile_size)
        max_bagfile_duration = self.parse_duration(self.args.max_bagfile_duration)

        if max_bagfile_size:
            output_storage_options.max_bagfile_size = int(max_bagfile_size) # in bytes
        if max_bagfile_duration:
            output_storage_options.max_bagfile_duration = int(max_bagfile_duration / 1e9)

        if max_bagfile_size > 0 or max_bagfile_duration > 0:
            print(f"Output bag split settings:")
            if max_bagfile_size > 0:
                print(f"  - max_bagfile_size: {max_bagfile_size} bytes ({max_bagfile_size / (1024*1024):.2f} MB)")
            if max_bagfile_duration > 0:
                print(f"  - max_bagfile_duration: {max_bagfile_duration} nanoseconds ({max_bagfile_duration / 1e9:.2f} seconds)")
        else:
            print(f"Output bag split settings: No splitting (single file)")
        
        writer.open(
            output_storage_options,
            ConverterOptions('', '')
        )

        # 4. Transfer Topic Metadata and Register New Topics
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
            
            # Write original data with ORIGINAL timestamp (preserves bag timeline)
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
            
            # Check what fields are available by testing if we can access them
            self.has_signal = False
            self.has_reflectivity = False
            self.has_near_ir = False
            self.has_range = False
            
            try:
                from ouster.sdk.core import ChanField
                # Try to access each field to verify availability
                if hasattr(ChanField, 'SIGNAL'):
                    self.lidar_scan.field(ChanField.SIGNAL)
                    self.has_signal = True
            except:
                pass
            
            try:
                from ouster.sdk.core import ChanField
                if hasattr(ChanField, 'REFLECTIVITY'):
                    self.lidar_scan.field(ChanField.REFLECTIVITY)
                    self.has_reflectivity = True
            except:
                pass
            
            try:
                from ouster.sdk.core import ChanField
                if hasattr(ChanField, 'NEAR_IR'):
                    self.lidar_scan.field(ChanField.NEAR_IR)
                    self.has_near_ir = True
            except:
                pass
            
            try:
                from ouster.sdk.core import ChanField
                if hasattr(ChanField, 'RANGE'):
                    self.lidar_scan.field(ChanField.RANGE)
                    self.has_range = True
            except:
                pass
            
            fields_info = []
            if self.has_signal:
                fields_info.append("SIGNAL")
            if self.has_reflectivity:
                fields_info.append("REFLECTIVITY")
            if self.has_near_ir:
                fields_info.append("NEAR_IR")
            if self.has_range:
                fields_info.append("RANGE")
            fields_str = ", ".join(fields_info) if fields_info else "None"
            
            self.pbar.write(f"[*] Ouster Metadata Initialized: {self.sensor_info.prod_line}")
            self.pbar.write(f"[*] Available fields: {fields_str}")
            self.pbar.write(f"[*] LidarScan info: {self.lidar_scan}")
        except Exception as e:
            self.pbar.write(f"[!] Metadata Error: {e}")

    def extract_packet_timestamp(self, data):
        """
        Extract ROS2 header timestamp from packet message.
        
        Args:
            data: Serialized packet message data
            
        Returns:
            int: Timestamp in nanoseconds, or 0 if extraction fails
        """
        try:
            from ouster_sensor_msgs.msg import PacketMsg
            pkt_msg = deserialize_message(data, PacketMsg)
            
            # Extract timestamp from ROS2 header
            if hasattr(pkt_msg, 'header') and hasattr(pkt_msg.header, 'stamp'):
                stamp = pkt_msg.header.stamp
                # Convert ROS2 timestamp to nanoseconds
                timestamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
                return timestamp_ns
            else:
                return 0
        except Exception as e:
            # If PacketMsg doesn't have header or deserialization fails
            return 0

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

    def create_cloud_with_fields(self, header, points, additional_fields):
        """
        Create a PointCloud2 message with XYZ and additional fields.
        
        Args:
            header: ROS Header
            points: Nx3 numpy array of XYZ coordinates
            additional_fields: List of tuples (field_name, field_data)
                              where field_data is a numpy array of length N
        
        Returns:
            PointCloud2 message
        """
        from sensor_msgs.msg import PointField
        
        # Define fields: X, Y, Z, and additional fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        offset = 12
        for field_name, _ in additional_fields:
            fields.append(PointField(name=field_name, offset=offset, datatype=PointField.FLOAT32, count=1))
            offset += 4
        
        # Create structured array
        point_step = offset
        cloud_data = np.zeros(len(points), dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
        ] + [(name, np.float32) for name, _ in additional_fields])
        
        cloud_data['x'] = points[:, 0]
        cloud_data['y'] = points[:, 1]
        cloud_data['z'] = points[:, 2]
        
        for field_name, field_data in additional_fields:
            cloud_data[field_name] = field_data
        
        # Create PointCloud2 message
        pc_msg = PointCloud2()
        pc_msg.header = header
        pc_msg.height = 1
        pc_msg.width = len(points)
        pc_msg.fields = fields
        pc_msg.is_bigendian = False
        pc_msg.point_step = point_step
        pc_msg.row_step = point_step * len(points)
        pc_msg.is_dense = True
        pc_msg.data = cloud_data.tobytes()
        
        return pc_msg

    def process_imu(self, data, tstamp, writer):
        try:
            # Extract timestamp from packet header for the message content
            header_timestamp = self.extract_packet_timestamp(data)
            # Use header timestamp for message stamp, fallback to bag timestamp
            msg_stamp = header_timestamp if header_timestamp != 0 else tstamp
            
            raw_buf = self.get_raw_buffer(data)
            imu_pkt = ImuPacket(self.packet_format.imu_packet_size)
            np.frombuffer(imu_pkt.buf, dtype=np.uint8)[:] = np.frombuffer(raw_buf, dtype=np.uint8)
            
            imu_msg = Imu()
            # Use extracted timestamp for message header
            imu_msg.header.stamp = rclpy.time.Time(nanoseconds=msg_stamp).to_msg()
            imu_msg.header.frame_id = self.args.imu_frame_id
            
            imu_msg.linear_acceleration.x = self.packet_format.imu_la_x(imu_pkt.buf) * 9.80665
            imu_msg.linear_acceleration.y = self.packet_format.imu_la_y(imu_pkt.buf) * 9.80665
            imu_msg.linear_acceleration.z = self.packet_format.imu_la_z(imu_pkt.buf) * 9.80665
            imu_msg.angular_velocity.x = math.radians(self.packet_format.imu_av_x(imu_pkt.buf))
            imu_msg.angular_velocity.y = math.radians(self.packet_format.imu_av_y(imu_pkt.buf))
            imu_msg.angular_velocity.z = math.radians(self.packet_format.imu_av_z(imu_pkt.buf))

            # CRITICAL: Write with ORIGINAL bag timestamp to preserve timeline
            writer.write(self.args.imu_topic, serialize_message(imu_msg), tstamp)
            self.imu_count += 1
            return True
        except Exception:
            return False

    def process_lidar(self, data, tstamp, writer):
        try:
            # Extract timestamp from packet header for the message content
            header_timestamp = self.extract_packet_timestamp(data)
            # Use header timestamp for message stamp, fallback to bag timestamp
            msg_stamp = header_timestamp if header_timestamp != 0 else tstamp
            
            raw_buf = self.get_raw_buffer(data)
            lidar_pkt = LidarPacket(self.packet_format.lidar_packet_size)
            dst = np.frombuffer(lidar_pkt.buf, dtype=np.uint8)
            src = np.frombuffer(raw_buf, dtype=np.uint8)
            dst[:len(src)] = src[:len(dst)]

            # Capture first packet timestamp when starting a new batch
            if self.first_packet_timestamp is None:
                self.first_packet_timestamp = msg_stamp

            # Check if batch is complete
            if self.scan_batcher(lidar_pkt, self.lidar_scan):
                xyz = self.xyzlut(self.lidar_scan)
                points = xyz.reshape(-1, 3)
                mask = np.any(points != 0, axis=1)
                valid_points = points[mask]

                if valid_points.size > 0:
                    header = Header()
                    # Use first packet timestamp for the completed scan
                    header.stamp = rclpy.time.Time(nanoseconds=self.first_packet_timestamp).to_msg()
                    header.frame_id = self.args.lidar_frame_id
                    
                    # Extract intensity and reflectivity if available
                    additional_fields = []
                    
                    if self.has_signal:
                        try:
                            from ouster.sdk.core import ChanField
                            signal = self.lidar_scan.field(ChanField.SIGNAL)
                            signal_flat = signal.reshape(-1)[mask]
                            additional_fields.append(('intensity', signal_flat.astype(np.float32)))
                        except Exception as e:
                            self.pbar.write(f"[!] Failed to extract SIGNAL: {e}")
                    
                    if self.has_reflectivity:
                        try:
                            from ouster.sdk.core import ChanField
                            reflectivity = self.lidar_scan.field(ChanField.REFLECTIVITY)
                            reflectivity_flat = reflectivity.reshape(-1)[mask]
                            additional_fields.append(('reflectivity', reflectivity_flat.astype(np.float32)))
                        except Exception as e:
                            self.pbar.write(f"[!] Failed to extract REFLECTIVITY: {e}")
                    
                    if self.has_near_ir:
                        try:
                            from ouster.sdk.core import ChanField
                            near_ir = self.lidar_scan.field(ChanField.NEAR_IR)
                            near_ir_flat = near_ir.reshape(-1)[mask]
                            additional_fields.append(('near_ir', near_ir_flat.astype(np.float32)))
                        except Exception as e:
                            self.pbar.write(f"[!] Failed to extract NEAR_IR: {e}")
                    
                    if self.has_range:
                        try:
                            from ouster.sdk.core import ChanField
                            range_data = self.lidar_scan.field(ChanField.RANGE)
                            range_flat = range_data.reshape(-1)[mask]
                            # Convert range from mm to meters
                            additional_fields.append(('range', (range_flat / 1000.0).astype(np.float32)))
                        except Exception as e:
                            self.pbar.write(f"[!] Failed to extract RANGE: {e}")
                    
                    # Create point cloud with additional fields
                    if additional_fields:
                        pc_msg = self.create_cloud_with_fields(header, valid_points, additional_fields)
                    else:
                        pc_msg = pc2.create_cloud_xyz32(header, valid_points)
                    
                    # CRITICAL: Write with ORIGINAL bag timestamp to preserve timeline
                    writer.write(self.args.points_topic, serialize_message(pc_msg), tstamp)
                    self.scan_count += 1
                
                # Reset for next batch
                self.first_packet_timestamp = None
                return True
            return False
        except Exception as e:
            return False

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Ouster Bag Converter - ROS2 Header Timestamps',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  # No splitting
  %(prog)s -i input.mcap -o output.mcap
  
  # Split by size
  %(prog)s -i input.mcap -o output.mcap --max-bagfile-size 500M
  %(prog)s -i input.mcap -o output.mcap --max-bagfile-size 2G
  
  # Split by duration
  %(prog)s -i input.mcap -o output.mcap --max-bagfile-duration 60s
  %(prog)s -i input.mcap -o output.mcap --max-bagfile-duration 5m
  
  # Split by both (whichever limit is reached first)
  %(prog)s -i input.mcap -o output.mcap --max-bagfile-size 500M --max-bagfile-duration 60s

Size suffixes: K/KB (kilobytes), M/MB (megabytes), G/GB (gigabytes)
Duration suffixes: s/sec (seconds), m/min (minutes), h/hour (hours)
        '''
    )
    parser.add_argument('--input-bag', '-i', required=True, help='Input bag file path')
    parser.add_argument('--output-bag', '-o', required=True, help='Output bag file path')
    parser.add_argument('--metadata-topic', default='/ouster/metadata', help='Metadata topic name')
    parser.add_argument('--lidar-packets-topic', default='/ouster/lidar_packets', help='Lidar packets topic name')
    parser.add_argument('--imu-packets-topic', default='/ouster/imu_packets', help='IMU packets topic name')
    parser.add_argument('--points-topic', default='/ouster/points', help='Output points topic name')
    parser.add_argument('--imu-topic', default='/ouster/imu', help='Output IMU topic name')
    parser.add_argument('--lidar-frame-id', default='os_lidar', help='Frame ID for lidar data')
    parser.add_argument('--imu-frame-id', default='os_imu', help='Frame ID for IMU data')
    parser.add_argument('--max-bagfile-size', type=str, default='0', 
                        help='Maximum bag file size (0 = no splitting). Examples: 500M, 2G, 1024K')
    parser.add_argument('--max-bagfile-duration', type=str, default='0',
                        help='Maximum bag file duration (0 = no splitting). Examples: 60s, 5m, 1h')
    
    args = parser.parse_args()
    rclpy.init()
    try:
        BagConverter(args).run()
    finally:
        rclpy.shutdown()