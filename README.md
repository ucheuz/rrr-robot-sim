# RRR Robot Tracking & Obstacle Avoidance

This project simulates a 3-joint planar robot arm (RRR manipulator) tracking a moving target in 2D space. It includes obstacle avoidance capabilities and compares performance across different control and update frequencies.

---

## Project Overview

The robot follows a moving target along a sine wave path. Joint angles are calculated using inverse kinematics (IK), and movement is visualised using Matplotlib. There are three modes you can run:

- **Task A**: High-frequency control (1kHz) with fast target updates (30Hz)
- **Task B**: Lower control (50Hz) with slower updates (5Hz)
- **Obstacle Avoidance**: Robot detects and avoids a red circular obstacle (or freezes)

---

## How to Set Up and Run

### 1. Clone the repo (or download the files)
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
```

### 5. Run the Simulation
Open `main.py` and choose the mode at the top:
```python
MODE = "task_a"       # Options: "task_a", "task_b", "obstacle"
```

Then run:
```bash
python main.py
```

---

## How It Works

- **Inverse Kinematics** (IK): Solved via gradient descent to minimise Euclidean distance from end-effector (EE) to the target.
- **Forward Kinematics** (FK): Uses vector sums from joint angles.
- **Obstacle Avoidance**:
  - Checks if the EE would enter the red circular zone.
  - If so, it tries a rerouted position away from the obstacle.
  - If rerouting fails, the robot freezes for that frame (visually shown as a break in the path).
- **Animation**: Displays arm joints, EE, target, obstacles, and violations.

---

## Files

- `main.py` — Run the simulation (choose the task here)
- `robot.py` — Contains FK, IK, and obstacle logic
- `simulation.py` — Generates sine wave target
- `visualise.py` — Animates the robot and plots paths
- `robot_tracking.mp4` — Output animation (if `save=True`)

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

  
## Author

**Uchemudi Uzoka**  

