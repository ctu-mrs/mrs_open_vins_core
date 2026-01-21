import re
import matplotlib.pyplot as plt
from collections import defaultdict
import numpy as np

def parse_log_file(filename):
    """Parse MSCKF log file and extract variables of interest."""
    bg_values = {"x": [], "y": [], "z": []}
    ba_values = {"x": [], "y": [], "z": []}
    timeoffset_values = []
    
    with open(filename, 'r') as f:
        lines = f.readlines()
    
    line_count = 0
    max_lines = 301000
    for line in lines:
        if max_lines > 0:
            line_count += 1
            if line_count >= max_lines:
                print(f"reached line {max_lines}")
                break

        # Parse bg (gyroscope bias)
        bg_match = re.search(r'bg = ([-\d.]+),([-\d.]+),([-\d.]+)', line)
        ba_match = re.search(r'ba = ([-\d.]+),([-\d.]+),([-\d.]+)', line)
        timeoffset_match = re.search(r'camera-imu timeoffset = ([-\d.]+)', line)
        
        # Only add if all three are present on the same line
        if bg_match and ba_match:
            bg_values["x"].append(float(bg_match.group(1)))
            bg_values["y"].append(float(bg_match.group(2)))
            bg_values["z"].append(float(bg_match.group(3)))
            
            ba_values["x"].append(float(ba_match.group(1)))
            ba_values["y"].append(float(ba_match.group(2)))
            ba_values["z"].append(float(ba_match.group(3)))
            
        if timeoffset_match:
            timeoffset_values.append(float(timeoffset_match.group(1)))
    
    # Create timestamps based on the number of measurements
    timestamps = np.arange(len(bg_values["x"]))
    
    return bg_values, ba_values, timeoffset_values, timestamps

def plot_results(bg_values, ba_values, timeoffset_values, timestamps):
    """Create separate matplotlib windows for each variable group."""
    
    # Figure 1: Gyroscope Bias (bg)
    fig1, ax1 = plt.subplots(figsize=(10, 6))
    ax1.plot(timestamps, bg_values["x"], marker='o', label='bg_x', linewidth=2)
    ax1.plot(timestamps, bg_values["y"], marker='s', label='bg_y', linewidth=2)
    ax1.plot(timestamps, bg_values["z"], marker='^', label='bg_z', linewidth=2)
    ax1.set_xlabel('Measurement Index')
    ax1.set_ylabel('Gyroscope Bias (rad/s)')
    ax1.set_title('Gyroscope Bias (bg) Over Time')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Figure 2: Accelerometer Bias (ba)
    fig2, ax2 = plt.subplots(figsize=(10, 6))
    ax2.plot(timestamps, ba_values["x"], marker='o', label='ba_x', linewidth=2)
    ax2.plot(timestamps, ba_values["y"], marker='s', label='ba_y', linewidth=2)
    ax2.plot(timestamps, ba_values["z"], marker='^', label='ba_z', linewidth=2)
    ax2.set_xlabel('Measurement Index')
    ax2.set_ylabel('Accelerometer Bias (m/s²)')
    ax2.set_title('Accelerometer Bias (ba) Over Time')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Figure 3: Camera-IMU Timeoffset
    fig3, ax3 = plt.subplots(figsize=(10, 6))
    ax3.plot(timestamps, timeoffset_values, marker='o', color='green', linewidth=2)
    ax3.set_xlabel('Measurement Index')
    ax3.set_ylabel('Time Offset (seconds)')
    ax3.set_title('Camera-IMU Time Offset Over Time')
    ax3.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # Parse the log file
    log_file = "serial_output.txt"  # Change this to your log file path
    
    try:
        bg_values, ba_values, timeoffset_values, timestamps = parse_log_file(log_file)
        
        # Print statistics
        print("=== Parsed Data Summary ===")
        print(f"Number of measurements: {len(timestamps)}")
        print(f"\nGyroscope Bias (bg):")
        print(f"  x: {np.mean(bg_values['x']):.6f} ± {np.std(bg_values['x']):.6f}")
        print(f"  y: {np.mean(bg_values['y']):.6f} ± {np.std(bg_values['y']):.6f}")
        print(f"  z: {np.mean(bg_values['z']):.6f} ± {np.std(bg_values['z']):.6f}")
        
        print(f"\nAccelerometer Bias (ba):")
        print(f"  x: {np.mean(ba_values['x']):.6f} ± {np.std(ba_values['x']):.6f}")
        print(f"  y: {np.mean(ba_values['y']):.6f} ± {np.std(ba_values['y']):.6f}")
        print(f"  z: {np.mean(ba_values['z']):.6f} ± {np.std(ba_values['z']):.6f}")
        
        print(f"\nCamera-IMU Timeoffset:")
        print(f"  Mean: {np.mean(timeoffset_values):.6f} seconds")
        print(f"  Std Dev: {np.std(timeoffset_values):.6f} seconds")
        
        # Create plots
        plot_results(bg_values, ba_values, timeoffset_values, timestamps)
        
    except FileNotFoundError:
        print(f"Error: Log file '{log_file}' not found.")
        print("Please update the 'log_file' variable with the correct path to your MSCKF log file.")