# RRR Robot Tracking & Obstacle Avoidance

This project simulates a 3-joint planar robotic arm (RRR manipulator) that follows a moving target in 2D space. It includes obstacle avoidance features and compares performance across different control and update frequencies.

---

## Project Overview

The robot tracks a target following a sine wave trajectory. Joint angles are calculated using inverse kinematics (IK), and the motion is visualised with Matplotlib. The system runs in three different modes:

- **Task A**: High-frequency control (1kHz) with fast target updates (30Hz)
- **Task B**: Lower control (50Hz) with slower updates (5Hz)
- **Obstacle Avoidance**: Robot detects and avoids a static circular obstacle, rerouting or freezing if needed

---

## How to Set Up and Run

### 1. Clone the repository (or download the files)
```bash
git clone https://github.com/ucheuz/rrr-robot-sim.git
cd rrr-robot-sim
```

### 2. Create a Virtual Environment
```bash
python3 -m venv venv
source venv/bin/activate        # macOS/Linux
venv\Scripts\activate.bat       # Windows
```

### 3. Install Required Packages
```bash
pip install -r requirements.txt
```

> Or manually:
```bash
pip install numpy matplotlib
```

### 4. Install ffmpeg (for saving .mp4 animations)
```bash
brew install ffmpeg    # macOS
# or use apt / choco depending on your OS
sudo apt update
sudo apt install ffmpeg -y
```

### 5. Run the Simulation
Edit the top of main.py to select the mode:
```python
MODE = "task_a"       # Options: "task_a", "task_b", "obstacle"
```

Then run:
```bash
python main.py
```

---

## Description of My Approach
The core idea was to simulate a 3-link planar robotic arm (RRR configuration) that tracks a moving target in 2D space, with a particular focus on accurate kinematic control and basic obstacle avoidance.

- **Forward Kinematics** (FK): Implemented using trigonometric relationships derived from the joint angles and link lengths. Each link’s position builds on the previous one
- **Inverse Kinematics** (IK): Solved numerically using gradient descent. I didn’t use closed-form solutions since the robot must constantly adapt to a dynamic target. Instead, the algorithm minimises the Euclidean distance between the end-effector and the target point. I used finite differences to estimate the gradient of the loss function with respect to each joint angle..

- **Obstacle Avoidance**: A static circular obstacle is placed near the tracking path. Before accepting a new movement, the system checks whether the end-effector would enter the obstacle region. If it does, the algorithm attempts to reroute the target slightly away from the obstacle in the opposite direction. If the reroute still leads to a violation, the robot simply holds its last safe position for that frame. This is a practical fallback I chose to avoid jittery or erratic movements near the boundary
- **Path Logging & Visualisation**: To reflect failed movements, I introduce NaN entries into the logged end-effector path so that the animation shows a visual break in the blue tracking line. I also added magenta dots to highlight any violation attempts, allowing me to analyse weak points in the algorithm visually.
  
- **Animation**: Displays arm joints, EE, target, obstacles, and violations.

---

## Files

- `main.py` — Run the simulation (choose the task here)
- `robot.py` — Contains FK, IK, and obstacle logic
- `simulation.py` — Generates sine wave target
- `visualise.py` — Animates the robot and plots paths
- `taska.png` -  Static visual of the result of the end effector path following the target path for Task A
- `taskb.png` -  Static visual of the result of the end effector path following the target path for Task B
- `obstacle.png` -  Static visual of the result of the end effector path following the target path with an obstacle margin and avoidance points
- `robot_tracking_task_a.mp4` — Output animation for Task A (if `save=True`)
- `robot_tracking_task_b.mp4` - Output animation for Task B (if `save=True`)
- `robot_tracking_obstacle.mp4` - Output animation for Obstacle Task (if `save=True`)

---

## System Requirements

- Python 3.8+
- macOS / Linux / Windows
- No GPU or special hardware needed
- Libraries:
  - `numpy`
  - `matplotlib`
  - `ffmpeg` (for video export)

---

## Results

### Task A – High Frequency Tracking
- **Control loop**: 1 kHz  
- **Target updates**: 30 Hz  
- **Final tracking error**: `0.0125`  
- **Animation**: `robot_tracking_task_a.mp4`

**Observation**:  
The robot follows the target smoothly and almost continuously. With high-frequency updates, there’s very little visible lag between the robot and the sine wave target.

---

### Task B – Low Frequency Tracking
- **Control loop**: 50 Hz  
- **Target updates**: 5 Hz  
- **Final tracking error**: `0.0130`  
- **Animation**: `robot_tracking_task_b.mp4`

**Observation**:  
With slower updates, movement appears more jumpy. The robot lags slightly behind the target, and the animation completes quicker due to fewer updates. Despite that, accuracy is only slightly worse.

**Takeaway**:  
Lower frequencies reduce smoothness but still maintain good tracking performance.

---
### Obstacle Task
- **Control loop**: 1 kHz  
- **Target updates**: 30 Hz
- **Final tracking error**: `0.1339`
- **Animation**: `robot_tracking_obstacle.mp4`

**Observation**:  
The robot almost always avoids the obstacle using rerouting or freezing, with a few trailing end effector lines. Path gaps and violation markers provide visual feedback of avoidance behaviour.

---

## Author
**Uchemudi Uzoka**  

