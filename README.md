# RRR Robot Tracking & Obstacle Avoidance

This project simulates a 3-joint planar robot arm (RRR manipulator) tracking a moving target in 2D space. It includes obstacle avoidance capabilities and can be used to compare performance across different control frequencies.

---

## Project Overview

The robot tracks a target that moves along a sine wave path. Joint angles are updated using inverse kinematics, and the robot's movement is visualised using Matplotlib. The simulation supports three modes:

- **Task A**: High-frequency control (1kHz) with fast target updates (30Hz)
- **Task B**: Lower control frequency (50Hz) with slower target updates (5Hz)
- **Obstacle Avoidance**: Includes a red circular obstacle. The robot reroutes or halts if it gets too close.

---

## How to Set Up and Run

### 1. Clone this repository and navigate to the directory:

   ```bash
   git clone https://github.com/ucheuz/rrr-robot-sim.git
   cd rrr-robot-sim

### 2. Create a Virtual Environment (recommended)
Open a terminal in the project folder and run:

```bash
python3 -m venv venv
source venv/bin/activate        # For macOS/Linux
venv\Scripts\activate.bat       # For Windows

###  3. Install Required Packages
pip install -r requirements.txt
brew install ffmpeg #For macOS
#OR
pip install numpy matplotlib
brew install ffmpeg #For macOS

### 4. Run the Simulation
Open main.py and set the mode at the top

MODE = "task_a"       # Options: "task_a", "task_b", "obstacle"

Then run the script
python main.py

## How it Works
Inverse Kinematics (IK): Solved using gradient descent on Euclidean loss between EE and target.
Forward Kinematics (FK): Sums vector links using joint angles.
Obstacle Avoidance:
    - Checks if proposed EE position enters the red circular zone
    - If so, reroutes to a safe offset position
    - If reroute also fails, the robot freezes and skips logging that frame
Animation: Robot arm, EE, target, obstacle, and violations are shown over time.


## Files
main.py — runs the simulation (edit here to choose the task)
robot.py — contains kinematics and obstacle detection
simulation.py — generates the sine wave target path
visualise.py — animates the robot and plots static summaries
robot_tracking.mp4 — optional animation output (if save=True in main.py)

## System Requirements
Python 3.8 or higher
macOS, Linux, or Windows
No GPU or special hardware needed
Requires ffmpeg to export MP4

Libraries:
numpy
matplotlib

## Notes
The animation will display in a pop-up window. To save it to an .mp4 file, set save=True in the animate_tracking() call inside main.py.
If the end-effector attempts to enter the obstacle, its motion is frozen and visualised as a break in the path.
Violation points (if any) are shown in magenta.

## Results
Task A – High Frequency Tracking
Control loop: 1 kHz
Target update rate: 30 Hz
Final tracking error: 0.0125
Animation: robot_tracking_task_a.mp4
What Was Seen:
The robot tracks the target very smoothly. Because both the control loop and the target update rate are fast, the end-effector stays close to the moving sine wave. There's barely any visible lag in the animation.

Task B – Lower Frequency Tracking
Control loop: 50 Hz
Target update rate: 5 Hz
Final tracking error: 0.0130
Animation: robot_tracking_task_b.mp4
What Was Seen:
With slower updates, both the target and the robot move in bigger jumps. The tracking still works well, but it looks a bit less smooth than Task A. The robot slightly lags behind the target at times, but the error increase is small. The animation also finishes faster since there are fewer target points to follow.
Takeaway:
Reducing control and update frequency makes the motion less fluid but doesn't significantly affect tracking accuracy.

## Author
Uchemudi Uzoka