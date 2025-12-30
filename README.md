# Offline Packet Converter

A small utility to convert and augment MCAP ROS2 bags containing raw Ouster lidar and IMU packets into sensor message topics (PointCloud2 and Imu) while preserving original topics. This repository includes a Python converter script and a Docker image definition for running the converter in a containerized ROS environment.

## What this does

- Reads an input MCAP bag and writes an output MCAP bag while copying all original topics.
- Converts raw Ouster lidar packets into `sensor_msgs/PointCloud2` messages and raw IMU packets into `sensor_msgs/Imu` messages.
- The converter preserves existing topics and registers new topics for points and IMU output.

## Prerequisites

- Docker must be installed. The Docker build relies on a base image `ros2-apparatus-base:jazzy` (used by `Dockerfile.converter`). If that base image isn't available locally, adapt the Dockerfile or obtain a compatible ROS 2 base image.

## Files of interest

- `docker_build/convert_bag_packets.py` — the converter script. Run with `python convert_bag_packets.py -i <in.mcap> -o <out.mcap>`.
- `docker_build/requirements.txt` — pip-installable Python deps (non-ROS).
- `Dockerfile.converter` — Dockerfile that builds a container capable of running the converter.
- `docker_build/docker-entrypoint.sh` — container entrypoint that runs the converter script.

## Running (local Python)

1. Ensure ROS 2 and the required ROS Python packages are installed and available in your environment.
2. Install non-ROS Python dependencies into a venv (optional but recommended):

## Offline Packet Converter — Docker usage

This repository contains a Docker-based converter to process Ouster packet bags. The README below explains the minimal Docker-only workflow.

1) Build everything with docker-compose (this builds the image and required pieces):

```bash
docker-compose build
```

2) Place the folder containing your input bags into the repository `./rosbags/` directory. For example, put your input bag(s) under `./rosbags/input_bag/`.

3) Run the converter with docker-compose (this mounts `./rosbags` into the container):

```bash
docker-compose run --rm converter --input /rosbags/input_bag/ --output /rosbags/output_bag/
```

Notes:
- The converter expects you to provide input and output paths inside the container; mounting the local `./rosbags` directory at `/rosbags` (as shown) is the simplest approach.


