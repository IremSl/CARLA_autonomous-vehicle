#  Autonomous Lane Keeping & Control in CARLA Simulator

![ROS 2](https://img.shields.io/badge/ROS2-Humble-red) ![CARLA](https://img.shields.io/badge/Simulator-CARLA-blue) ![OpenCV](https://img.shields.io/badge/Vision-OpenCV-green) ![Status](https://img.shields.io/badge/Status-Active-success)

##  Overview

This project implements a complete **Autonomous Lane Keeping System** using **ROS 2** and the **CARLA Simulator**. It is the final capstone project of my training at the **ITU Solar Car Team**.

The system is designed to autonomously navigate a vehicle through a winding track by processing raw camera data to detect lane lines and applying control algorithms to steer the vehicle. It features a dual-mode controller (Straight vs. Turn) and a GNSS-based stopping mechanism.

---

##  Features

- **Computer Vision Pipeline:** Real-time lane detection using OpenCV (Canny Edge, Hough Transform).
- **Robust Lane Filtering:** Filters lines based on slope and position to distinguish left/right lanes from noise.
- **Dual-Mode Control Logic:**
  - **Lane Centering (PID):** Keeps the car centered on straight roads using a P-controller.
  - **Turn Handling:** Detects sharp curves via slope analysis and switches to a specific "Turn Mode" with adjusted throttle and steering.
- **GNSS Navigation:** Monitors the vehicle's global position and autonomously brakes when the target destination is reached.
- **ROS 2 Architecture:** Modular design with separate nodes for Perception (`lane_detection`) and Control (`pure_pursuit_controller`).

---

##  System Architecture

The project consists of two main ROS 2 nodes:

1.  **`lane_detection_node`**: Handles image processing.
    * Subscribes to CARLA RGB Camera.
    * Publishes Lane Center (`/lane_center`) and Lane Slopes (`/left_slope`, `/right_slope`).
2.  **`pure_pursuit_controller`**: Handles vehicle dynamics.
    * Subscribes to lane data and GNSS.
    * Calculates Steering, Throttle, and Brake.
    * Publishes to CARLA Vehicle Control (`/carla/ego_vehicle/vehicle_control_cmd`).

---

##  Installation & Build

### Prerequisites
* **ROS 2** (Humble/Foxy)
* **CARLA Simulator** (0.9.x)
* **carla_ros_bridge** package
* **OpenCV** (`libopencv-dev`)

### Build Instructions

1.  Clone the repository to your ROS 2 workspace `src` folder:
    ```bash
    cd ~/ros2_ws/src
    git clone <your-repo-link>
    ```

2.  Build the package using `colcon`:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select lane_detection
    ```

3.  Source the workspace:
    ```bash
    source install/setup.bash
    ```

---

##  Node Details & Algorithms

### 1. Perception: Lane Detection Node
**File:** `src/lane_detection.cpp`

This node processes the raw image stream to understand the road geometry.

* **Preprocessing:** Applies Gaussian Blur to reduce noise and converts the image to Grayscale.
* **Edge Detection:** Uses **Canny Edge Detection** to find high-contrast boundaries.
* **ROI Masking:** Focuses only on the road area, ignoring the sky and surroundings.
* **Line Extraction:** Uses **Probabilistic Hough Transform (`HoughLinesP`)** to detect line segments.
* **Classification:**
    * Lines are classified as **Left** or **Right** based on their slope (negative vs. positive) and position in the image.
    * Noise removal: Lines with slopes that are too flat or too vertical are discarded.
* **Output:** Calculates the visual center of the lane and the average slope of the curves.

**Visualization Output:**
*(Below is an example of the detected lane lines and the calculated center point)*

![](https://github.com/user-attachments/assets/PLACEHOLDER_FOR_LANE_IMAGE)
*(Please upload a screenshot of your OpenCV window showing lines and alignment text here)*

---

### 2. Control: Pure Pursuit Controller
**File:** `src/pure_pursuit_controller.cpp`

This node acts as the "brain" of the vehicle, deciding how to steer based on perception data.

#### Control Logic
* **Straight Road (P-Controller):**
    * If the slopes indicate a straight road, the system calculates the error between the *Image Center* and the *Lane Center*.
    * `Steer = -Kp * Error` (Proportional Control).
* **Turn Mode (State Machine):**
    * The system monitors the slope of the lane lines.
    * **Hysteresis:** If the slope exceeds `turn_min_slope` for a specific duration (`entry_hold`), the car enters **Turn Mode**.
    * In Turn Mode, the car slows down (`throttle` reduced) and applies a sharper steering angle.
    * It exits Turn Mode only when the road straightens out for a set duration (`exit_hold`).

#### GNSS Stopping Mechanism
* The node subscribes to `/carla/ego_vehicle/gnss`.
* If the vehicle reaches the target coordinates (`Target Lat: 0.167724`, `Target Lon: 0.104306`) within a specific tolerance, it applies full brakes (`brake = 1.0`) and shuts down the node.

---

##  Configuration (ROS Parameters)

You can tune the vehicle's behavior by changing parameters in `pure_pursuit_controller.cpp` or via launch files:

| Parameter | Default | Description |
| :--- | :--- | :--- |
| `base_throttle` | `0.3` | Cruising speed throttle value. |
| `min_throttle` | `0.05` | Minimum throttle during sharp turns. |
| `turn_min_slope` | `0.1` | Threshold to detect a curve start. |
| `center_Kp` | `0.0025` | P-Controller gain for steering. |
| `entry_hold` | `5` | Frames required to confirm a turn (noise filtering). |

---

##  Usage

1.  **Start CARLA Simulator:**
    ```bash
    ./CarlaUE4.sh
    ```

2.  **Launch CARLA ROS Bridge (and spawn the vehicle):**
    ```bash
    ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py
    ```

3.  **Run the Lane Detection Node:**
    ```bash
    ros2 run lane_detection lane_detection_node
    ```

4.  **Run the Controller Node:**
    ```bash
    ros2 run lane_detection pure_pursuit_controller
    ```

---

##  Contact

For any issues or questions, please reach out:

**Irem Aslan**  Email: iremaslan0420@hotmail.com  

---
*Developed as part of the ITU Solar Car Team Autonomous Driving Training Program.*
