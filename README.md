# Pick-and-Place Application with Robotic Arm and OpenCV

## Overview
This project implements a pick-and-place application using a Universal Robotics robotic arm controlled via OpenCV. The system detects objects of three different colors (red, green, and blue) and guides the robotic arm to pick and place them at predefined locations.

## Features
- **Color-Based Object Detection**: Identifies red, green, and blue objects using HSV color segmentation.
- **Camera Calibration**: Aligns camera coordinates with the robot's workspace for precise movements.
- **Robotic Arm Control**: Controls the robotic arm via RTDE protocol for smooth and accurate operations.
- **Pick-and-Place Operations**: Moves detected objects to designated positions based on their color.

## Dependencies
- Python 3.x
- OpenCV
- NumPy
- RTDE (Real-Time Data Exchange) Python Interface

## Usage
### Setup
1. Connect the Universal Robotics robotic arm to your network.
2. Update the IP address in the script:
   ```python
   self.arm = arm_control(ip="192.168.1.100")
   ```
3. Ensure the camera is connected and accessible.

### Run the Program
```bash
python main.py
```

## Code Structure
### ur3.py
- **Initialization**: Sets up the camera and robotic arm.
- **Detection Loop**: Continuously detects objects based on HSV color ranges.
- **Command Execution**: Guides the robotic arm to pick and place objects based on their color.

### arm_control.py
- Establishes a connection with the robotic arm using RTDE protocol.
- Supports movement commands like `move()`, `grab()`, `release()`, and `reset()`.

### detection.py
- Processes images to detect objects based on color segmentation.
- Computes object positions and maps them to the robot's workspace.

## Key Functions
- **get_cnt(img, thr=50)**: Extracts contours from a binary image.
- **get_point(cnt)**: Computes the centroid of detected contours.
- **to_red(point)**, **to_green(point)**, **to_blue(point)**: Moves detected objects to predefined positions based on color.

## Notes
- Press 'Q' to quit the application.
- Adjust HSV thresholds for color detection based on lighting conditions.
- Update the IP address of the robotic arm if different.

## Example Output
The program highlights detected objects in the camera feed and guides the robotic arm to pick and place them based on color.
