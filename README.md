# Ouster Offline Packet Converter

A Python utility for converting and augmenting MCAP ROS2 bags containing raw Ouster lidar and IMU packets into standard sensor message topics (PointCloud2 and Imu) while preserving all original topics and data.

## Overview

This converter processes ROS2 MCAP bags recorded from Ouster sensors, transforming proprietary packet formats into standard ROS sensor messages. It maintains the complete original bag content while adding new processed topics, making the data accessible to standard ROS tools and algorithms.

### Key Features

- **Non-destructive conversion**: All original topics are preserved in the output bag
- **Dual message generation**: Creates both `sensor_msgs/PointCloud2` and `sensor_msgs/Imu` messages
- **Rich point cloud data**: Includes XYZ coordinates plus intensity, reflectivity, near-IR, and range fields (when available)
- **Timestamp preservation**: Maintains original ROS2 header timestamps from packet data
- **Bag splitting**: Optional output bag splitting by size or duration
- **Docker-based deployment**: Containerized execution for consistent, reproducible processing

## What It Does

1. **Reads** an input MCAP bag containing Ouster sensor data
2. **Copies** all existing topics to the output bag unchanged
3. **Converts** raw lidar packets (`/ouster/lidar_packets`) to PointCloud2 messages
4. **Converts** raw IMU packets (`/ouster/imu_packets`) to Imu messages
5. **Registers** new topics for converted data (default: `/ouster/points` and `/ouster/imu`)
6. **Optionally splits** output bags based on size or duration constraints

### Point Cloud Features

The converter extracts and includes multiple data channels when available:
- **XYZ coordinates**: 3D point positions
- **Intensity**: Signal strength (from SIGNAL field)
- **Reflectivity**: Surface reflectivity measurements
- **Near-IR**: Near-infrared channel data
- **Range**: Distance measurements (converted from mm to meters)

## Prerequisites

### For Docker Usage (Recommended)
- Docker installed and running
- `docker-compose` (or Docker Compose V2)

### For Local Python Usage
- ROS 2 (tested with Jazzy)
- Python 3.8+
- Required ROS packages: `rclpy`, `rosbag2_py`, `sensor_msgs`, `sensor_msgs_py`, `ouster_sensor_msgs`
- Ouster Python SDK (`ouster-sdk`)
- Additional Python packages: `numpy`, `tqdm`

## Repository Structure

```
.
├── docker_build/
│   ├── requirements.txt           # Python dependencies
│   ├── entrypoint.sh              # Base container entrypoint
│   └── docker-entrypoint.sh       # Converter container entrypoint
├── scripts/
│   └── convert_bag_packets.py     # Main converter script
├── rosbags/                       # Mount point for bag files
├── Dockerfile.converter           # Docker image definition
├── docker-compose.yml             # Docker Compose configuration
└── README.md                      # This file
```

## Usage

### Docker Method (Recommended)

#### 1. Build the Container

```bash
docker-compose build
```

This builds the converter image with all necessary dependencies.

#### 2. Prepare Your Bag Files

Place your input bag(s) in the `./rosbags/` directory:

```bash
./rosbags/
└── my_input_bag/
    └── my_input_bag.mcap
```

#### 3. Run the Converter

Basic conversion (no splitting):

```bash
docker-compose run --rm converter \
  --input-bag /rosbags/my_input_bag \
  --output-bag /rosbags/my_output_bag
```

Split by size (500 MB chunks):

```bash
docker-compose run --rm converter \
  --input-bag /rosbags/my_input_bag \
  --output-bag /rosbags/my_output_bag \
  --max-bagfile-size 500M
```

Split by duration (60 second chunks):

```bash
docker-compose run --rm converter \
  --input-bag /rosbags/my_input_bag \
  --output-bag /rosbags/my_output_bag \
  --max-bagfile-duration 60s
```

Split by both (whichever limit is reached first):

```bash
docker-compose run --rm converter \
  --input-bag /rosbags/my_input_bag \
  --output-bag /rosbags/my_output_bag \
  --max-bagfile-size 500M \
  --max-bagfile-duration 60s
```

#### 4. Access Output

Your converted bag(s) will be in `./rosbags/my_output_bag/`.

### Local Python Method

#### 1. Set Up Environment

```bash
# Create and activate virtual environment (recommended)
python3 -m venv venv
source venv/bin/activate

# Install non-ROS dependencies
pip install -r docker_build/requirements.txt
```

#### 2. Run the Converter

```bash
python3 docker_build/convert_bag_packets.py \
  -i /path/to/input.mcap \
  -o /path/to/output.mcap
```

## Command Line Arguments

| Argument | Short | Default | Description |
|----------|-------|---------|-------------|
| `--input-bag` | `-i` | *required* | Input bag file path |
| `--output-bag` | `-o` | *required* | Output bag file path |
| `--metadata-topic` | | `/ouster/metadata` | Metadata topic name |
| `--lidar-packets-topic` | | `/ouster/lidar_packets` | Lidar packets topic name |
| `--imu-packets-topic` | | `/ouster/imu_packets` | IMU packets topic name |
| `--points-topic` | | `/ouster/points` | Output points topic name |
| `--imu-topic` | | `/ouster/imu` | Output IMU topic name |
| `--lidar-frame-id` | | `os_lidar` | Frame ID for lidar data |
| `--imu-frame-id` | | `os_imu` | Frame ID for IMU data |
| `--max-bagfile-size` | | `0` | Maximum bag file size (0 = no splitting) |
| `--max-bagfile-duration` | | `0` | Maximum bag file duration (0 = no splitting) |

### Size Format

Size values support these suffixes:
- **K** or **KB**: Kilobytes (1024 bytes)
- **M** or **MB**: Megabytes (1024² bytes)
- **G** or **GB**: Gigabytes (1024³ bytes)
- No suffix: Bytes

Examples: `500M`, `2G`, `1024K`, `1073741824`

### Duration Format

Duration values support these suffixes:
- **s** or **sec**: Seconds
- **m** or **min**: Minutes
- **h** or **hour**: Hours
- No suffix: Seconds

Examples: `60s`, `5m`, `1h`, `90`

## Examples

### Convert with Custom Topics

```bash
docker-compose run --rm converter \
  --input-bag /rosbags/raw_data \
  --output-bag /rosbags/processed_data \
  --lidar-packets-topic /custom/lidar \
  --points-topic /custom/points
```

### Convert with Custom Frame IDs

```bash
docker-compose run --rm converter \
  -i /rosbags/input \
  -o /rosbags/output \
  --lidar-frame-id sensor_frame \
  --imu-frame-id imu_frame
```

### Large Dataset Processing (2GB chunks)

```bash
docker-compose run --rm converter \
  -i /rosbags/large_dataset \
  -o /rosbags/large_dataset_converted \
  --max-bagfile-size 2G
```

## Technical Details

### Timestamp Handling

- **Message stamps**: Use timestamps extracted from ROS2 packet headers
- **Bag timeline**: Original bag timestamps are preserved for all messages
- **Scan timing**: Point clouds use the timestamp from the first packet in each complete scan

### Data Processing

- **Lidar**: Batches packets into complete scans using Ouster's ScanBatcher
- **Point clouds**: Filters zero points and includes all available sensor fields
- **IMU**: Converts accelerations to m/s² and angular velocities to rad/s
- **Coordinate transform**: Uses Ouster's XYZLut for efficient point cloud generation

### Splitting Behavior

When splitting is enabled:
- Output bags are numbered sequentially (e.g., `output_0.mcap`, `output_1.mcap`)
- Splits occur when **either** size or duration limit is reached
- All topics continue seamlessly across split boundaries
- Original message order and timestamps are preserved

## Troubleshooting

### Missing Base Image Error

If you encounter `ros2-apparatus-base:jazzy` not found:

1. Modify `Dockerfile.converter` to use an available ROS2 Jazzy base image:
   ```dockerfile
   FROM ros:jazzy
   ```

2. Or build/pull the required base image separately

### Import Errors

Ensure all ROS2 packages are sourced:
```bash
source /opt/ros/jazzy/setup.bash
```

### Permission Issues (Docker)

If output files have incorrect permissions:
```bash
# Run with your user ID
docker-compose run --rm --user $(id -u):$(id -g) converter ...
```

## Dependencies

### ROS 2 Packages
- `rclpy` - ROS 2 Python client library
- `rosbag2_py` - ROS 2 bag recording/playback
- `sensor_msgs` - Standard sensor message types
- `sensor_msgs_py` - Sensor message utilities
- `ouster_sensor_msgs` - Ouster-specific message definitions

### Python Packages
- `ouster-sdk` - Ouster sensor SDK for packet parsing
- `numpy` - Numerical computing
- `tqdm` - Progress bar visualization

## Support

For issues related to:
- **Ouster SDK**: Visit [Ouster SDK documentation](https://static.ouster.dev/sdk-docs/)
- **ROS 2**: Visit [ROS 2 documentation](https://docs.ros.org/)
- **This converter**: Open an issue in this repository