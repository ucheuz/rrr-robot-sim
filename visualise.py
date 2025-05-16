import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from matplotlib.patches import Circle

# Animate RRR robot tracking a moving target over time
def animate_tracking(ee_path, target_path, theta_history, L=1.0, interval=10, save=False, filename="robot_tracking.mp4", show_obstacle=True):

    # Given joint angles, compute the positions of each joint and end-effector
    def get_joint_positions(theta, L):
        theta1, theta2, theta3 = theta
        x0, y0 = 0, 0
        x1 = L * np.cos(theta1)
        y1 = L * np.sin(theta1)
        x2 = x1 + L * np.cos(theta1 + theta2)
        y2 = y1 + L * np.sin(theta1 + theta2)
        x3 = x2 + L * np.cos(theta1 + theta2 + theta3)
        y3 = y2 + L * np.sin(theta1 + theta2 + theta3)
        return [(x0, y0), (x1, y1), (x2, y2), (x3, y3)]

    ee_path = np.array(ee_path)
    target_path = np.array(target_path)

    # Filter NaNs for setting axis limits
    valid_ee = ee_path[~np.isnan(ee_path[:, 0])]
    all_x = np.concatenate([target_path[:, 0], ee_path[~np.isnan(ee_path[:, 0]), 0]])
    all_y = np.concatenate([target_path[:, 1], ee_path[~np.isnan(ee_path[:, 1]), 1]])
    x_margin = 0.4
    y_margin = 0.4

    # Set up the plot
    fig, ax = plt.subplots()
    ax.set_title("RRR Robot Tracking Animation")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.grid(True)
    ax.axis("equal")

    ax.set_xlim(np.min(all_x) - x_margin, np.max(all_x) + x_margin)
    ax.set_ylim(np.min(all_y) - y_margin, np.max(all_y) + y_margin)


    # Obstacle settings
    obstacle_center = (2 * L, 0.5 * L)
    obstacle_radius = L / 8
    if show_obstacle:
        obstacle_patch = Circle(obstacle_center, obstacle_radius, color='red', alpha=0.3, label='Obstacle')
        ax.add_patch(obstacle_patch)

    # Plot elements
    target_dot, = ax.plot([], [], 'go', label='Target')
    ee_dot, = ax.plot([], [], 'bo', label='End-Effector')
    target_line, = ax.plot([], [], '--g', linewidth=1)
    ee_line, = ax.plot([], [], '--b', linewidth=1)
    violation_dot, = ax.plot([], [], 'mo', label='Violation', markersize=6)

    ax.legend()

    # Initialisation function for the animation
    def init():
        target_dot.set_data([], [])
        ee_dot.set_data([], [])
        target_line.set_data([], [])
        ee_line.set_data([], [])
        joint_line.set_data([], [])
        violation_dot.set_data([], [])
        return target_dot, ee_dot, target_line, ee_line, joint_line, violation_dot

    # Update function called for each frame
    def update(frame):
        theta = theta_history[frame]
        joints = get_joint_positions(theta, L)

        # Update target and end-effector dots
        target_dot.set_data([target_path[frame, 0]], [target_path[frame, 1]])
        ee_dot.set_data([ee_path[frame, 0]], [ee_path[frame, 1]])

        # Draw the target path and EE path up to current frame
        target_line.set_data(target_path[:frame+1, 0], target_path[:frame+1, 1])
        valid_indices = ~np.isnan(ee_path[:frame+1, 0])
        ee_line.set_data(ee_path[:frame+1, 0][valid_indices], ee_path[:frame+1, 1][valid_indices])

        # Draw robot joints and links
        joint_coords = np.array(joints)
        joint_line.set_data(joint_coords[:, 0], joint_coords[:, 1])

        # Highlight if EE enters the obstacle region
        if show_obstacle and not np.any(np.isnan(ee_path[frame])):
            dist_to_obstacle = np.linalg.norm(ee_path[frame] - np.array(obstacle_center))
            if dist_to_obstacle < obstacle_radius:
                violation_dot.set_data([ee_path[frame, 0]], [ee_path[frame, 1]])
            else:
                violation_dot.set_data([], [])
        else:
            violation_dot.set_data([], [])

        return target_dot, ee_dot, target_line, ee_line, joint_line, violation_dot

    joint_line, = ax.plot([], [], 'ko-', lw=3, label="Robot Arm")

    # Create the animation
    ani = animation.FuncAnimation(
        fig, update, frames=len(ee_path),
        init_func=init, blit=True, interval=interval, repeat=False
    )

    # Save or show the animation
    if save:
        print("Saving animation to MP4...")
        Writer = animation.writers['ffmpeg']
        writer = Writer(fps=1000 // interval, metadata=dict(artist='Uche'), bitrate=1800)
        ani.save(filename, writer=writer)
        print(f"Saved animation as {filename}")
    else:
        plt.show()
