import numpy as np
import matplotlib.pyplot as plt
from robot import forward_kinematics, ik_solver, in_obstacle
from simulation import target_trajectory
from visualise import animate_tracking

# ========== CONFIGURATION ==========
# Choose the simulation mode: 'task_a', 'task_b', or 'obstacle'
MODE = "task_b"

L = 1.0  # # Length of each robot arm link
theta = np.zeros(3)  # Initial joint angles (3-DOF planar arm)

# Set time step and duration based on the selected task
if MODE == "task_a":
    dt = 0.001
    target_dt = 1 / 30.0
    total_time = 2.0
elif MODE == "task_b":
    dt = 1 / 50.0
    target_dt = 1 / 5.0
    total_time = 5.0
elif MODE == "obstacle":
    dt = 0.001
    target_dt = 1 / 30.0
    total_time = 2.0
else:
    raise ValueError("Invalid mode selected. Use 'task_a', 'task_b', or 'obstacle'.")

# Define the obstacle for "obstacle" mode
obstacle_center = np.array([2 * L, 0.5 * L])
obstacle_radius = L / 8
OBSTACLE_MARGIN = 0.03  # Safety margin around obstacle

# Initialize data storage
theta_history = [] # Joint angles over time
obstacle_hits = 0 # Counter for obstacle avoidance
obstacle_positions = [] # EE positions that triggered avoidance
ee_path = [] # End-effector (EE) trajectory
target_path = [] # Desired target trajectory (sampled)
true_target_path = []  # Ground truth continuous target path

# Simulation time trackers
time_elapsed = 0.0
next_target_time = 0.0
current_target = target_trajectory(0, L) # Starting target position

# Initial safety check for obstacle mode
if MODE == "obstacle" and in_obstacle(forward_kinematics(theta, L), L, margin=OBSTACLE_MARGIN):
    print("⚠️ Initial position inside obstacle — adjusting")
    theta += np.random.uniform(-0.2, 0.2, size=3)

# ========== MAIN SIMULATION LOOP ==========
while time_elapsed < total_time:
    # Log the true target path for error calculation
    true_target_path.append(target_trajectory(time_elapsed, L))

    # Sample a new target at fixed intervals
    if time_elapsed >= next_target_time:
        current_target = target_trajectory(time_elapsed, L)
        next_target_time += target_dt

    # Use inverse kinematics to compute next joint angles
    proposed_theta = ik_solver(current_target, theta, L)
    proposed_ee_pos = forward_kinematics(proposed_theta, L)

    print(f"t={time_elapsed:.3f}s | EE = {proposed_ee_pos}, dist to obstacle = {np.linalg.norm(proposed_ee_pos - obstacle_center):.4f}")

    # Obstacle avoidance logic (only in "obstacle" mode)
    if MODE == "obstacle" and in_obstacle(proposed_ee_pos, L):
        print(f"⚠️ Obstacle detected at t={time_elapsed:.2f}s — rerouting.")
        obstacle_hits += 1
        obstacle_positions.append(proposed_ee_pos.copy())

        # Move target away from the obstacle in the opposite direction
        direction = proposed_ee_pos - obstacle_center
        norm = np.linalg.norm(direction)
        direction = direction / norm if norm > 1e-6 else np.array([0, 1])
        rerouted_target = proposed_ee_pos + direction * (obstacle_radius + 0.05)

        # Try solving IK again for rerouted target
        rerouted_theta = ik_solver(rerouted_target, theta, L)
        rerouted_ee = forward_kinematics(rerouted_theta, L)

        # Only update if new EE position is safe
        if not in_obstacle(rerouted_ee, L, margin=OBSTACLE_MARGIN):
            theta = rerouted_theta
            ee_pos = rerouted_ee
            record_this_point = True
        else:
            ee_pos = forward_kinematics(theta, L) # Stick to current position
            record_this_point = False
    else:
        # No obstacle detected, update state as usual
        theta = proposed_theta
        ee_pos = proposed_ee_pos
        record_this_point = True

    # Record results for this timestep
    theta_history.append(theta.copy())
    ee_path.append(ee_pos.copy() if record_this_point else np.array([np.nan, np.nan]))
    target_path.append(current_target.copy())
    time_elapsed += dt

print(f"\n✅ Simulation complete. Obstacle avoided {obstacle_hits} times.\n")

# ========== VISUALISATION ==========
# Generate animation based on the selected mode
if MODE == "obstacle":
    animate_tracking(ee_path, target_path, theta_history=theta_history, L=L, save=True, filename="robot_tracking_obstacle.mp4", show_obstacle=True)
elif MODE == "task_a":
    animate_tracking(ee_path, target_path, theta_history=theta_history, L=L, save=True, filename="robot_tracking_task_a.mp4", show_obstacle=False)
elif MODE == "task_b":
    animate_tracking(ee_path, target_path, theta_history=theta_history, L=L, save=True, filename="robot_tracking_task_b.mp4", show_obstacle=False)
else:
    raise ValueError("Invalid mode selected. Use 'task_a', 'task_b', or 'obstacle'.")

# ========== FINAL ERROR ==========
# Compute the final distance between EE and the last target point
final_error = np.linalg.norm(ee_path[-1] - target_path[-1])
print(f"Final tracking error: {final_error:.4f}")

# ========== PLOTS ==========
ee_np = np.array(ee_path)
target_np = np.array(target_path)

plt.figure()
plt.plot(ee_np[:, 0], ee_np[:, 1], 'b-', alpha=0.7, label="End-Effector Path")
plt.plot(target_np[:, 0], target_np[:, 1], 'g--', label="Target Path")

if MODE == "obstacle":
    # Show obstacle and avoidance info
    plt.scatter(*obstacle_center, c='red', label="Obstacle Center")
    circle = plt.Circle(obstacle_center, obstacle_radius, color='red', alpha=0.3, label="Obstacle Margin")
    plt.gca().add_patch(circle)

    if obstacle_positions:
        obs_pos = np.array(obstacle_positions)
        plt.plot(obs_pos[:, 0], obs_pos[:, 1], 'rx', label="Avoidance Points")

    # Check for any violations where EE entered the obstacle
    violating = np.linalg.norm(ee_np - obstacle_center, axis=1) < (obstacle_radius - 1e-6)
    if np.any(violating):
        print(f"⚠️ {np.sum(violating)} points violated the obstacle boundary.")
        plt.scatter(ee_np[violating, 0], ee_np[violating, 1], color='magenta', label="⚠️ Violations")

plt.axis('equal')
plt.legend()
plt.title("EE Path vs Obstacle" if MODE == "obstacle" else "EE vs Target Path")
plt.grid()
plt.show()

# ========== ERROR PLOT ==========
# Compute error over time relative to continuous ground truth
errors_true = [np.linalg.norm(ee - t) for ee, t in zip(ee_path, true_target_path)]
print(f"Length of ee_path: {len(ee_path)}")
print(f"Length of true_target_path: {len(true_target_path)}")
print(f"First 5 errors: {errors_true[:5]}")

plt.plot(errors_true)
plt.title("Tracking Error Over Time (vs True Sine Wave)")
plt.xlabel("Timestep")
plt.ylabel("Distance Error")
plt.grid(True)
plt.show()
