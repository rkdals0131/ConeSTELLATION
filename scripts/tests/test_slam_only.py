#!/usr/bin/env python3

import subprocess
import time
import signal
import sys
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def run_slam_test():
    print("Starting inter-landmark factor test...")
    
    # Find workspace root dynamically
    current_file = Path(__file__).resolve()
    workspace_root = current_file.parent.parent.parent.parent.parent  # Navigate up to ros2_ws
    setup_bash = workspace_root / "install" / "setup.bash"
    
    # Start dummy publisher
    dummy_proc = subprocess.Popen(
        ["bash", "-c", f"source {setup_bash} && ros2 run cone_stellation dummy_publisher_node.py"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )
    
    print("Dummy publisher started, waiting for initialization...")
    time.sleep(3)
    
    # Start SLAM node and capture output
    # Get package share directory for config file
    try:
        cone_stellation_share = get_package_share_directory('cone_stellation')
        config_file = Path(cone_stellation_share) / 'config' / 'slam_config.yaml'
    except:
        # Fallback to source directory if package not installed
        config_file = workspace_root / 'src' / 'cone_stellation' / 'config' / 'slam_config.yaml'
    
    slam_proc = subprocess.Popen(
        ["bash", "-c", f"source {setup_bash} && ros2 run cone_stellation cone_slam_node --ros-args --params-file {config_file}"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )
    
    print("\nMonitoring SLAM output for inter-landmark factors...")
    print("=" * 60)
    
    start_time = time.time()
    inter_landmark_count = 0
    
    try:
        while time.time() - start_time < 20:  # Run for 20 seconds
            line = slam_proc.stdout.readline()
            if line:
                # Filter for relevant lines
                if any(keyword in line for keyword in ["Frame", "observed_landmark_ids", "Creating inter-landmark", "Created inter-landmark", "Added promoted"]):
                    print(line.strip())
                    if "Created inter-landmark factor" in line:
                        inter_landmark_count += 1
                        
    except KeyboardInterrupt:
        pass
    
    print("\n" + "=" * 60)
    print(f"Test completed. Inter-landmark factors created: {inter_landmark_count}")
    
    # Clean up
    slam_proc.terminate()
    dummy_proc.terminate()
    time.sleep(1)
    slam_proc.kill()
    dummy_proc.kill()

if __name__ == "__main__":
    run_slam_test()