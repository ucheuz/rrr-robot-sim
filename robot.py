import numpy as np

# Compute the 2D forward kinematics for a 3-link planar robot arm
def forward_kinematics(theta, L):
    theta1, theta2, theta3 = theta
    # Sum the x and y components of each link based on joint angles
    x = L * (np.cos(theta1) + np.cos(theta1 + theta2) + np.cos(theta1 + theta2 + theta3))
    y = L * (np.sin(theta1) + np.sin(theta1 + theta2) + np.sin(theta1 + theta2 + theta3))
    return np.array([x, y])

# Compute Euclidean distance between end-effector and target
def loss(theta, target, L):
    ee = forward_kinematics(theta, L)
    return np.linalg.norm(ee - target)

# Inverse kinematics solver using numerical gradient descent
def ik_solver(target, theta_init, L, lr=0.01, steps=200):
    theta = theta_init.copy()

    # Define obstacle position and size
    obstacle_center = np.array([2 * L, 0.5 * L])
    obstacle_radius = L / 8

    for _ in range(steps):
        ee_pos = forward_kinematics(theta, L)
        dist = np.linalg.norm(ee_pos - obstacle_center)

        # Reduce step size when near the obstacle to avoid collisions
        if dist < 2 * obstacle_radius:
            scaled_lr = lr * (dist / (2 * obstacle_radius))
        else:
            scaled_lr = lr

        # Estimate gradient using differences
        grad = np.zeros(3)
        for i in range(3):
            delta = np.zeros(3)
            delta[i] = 1e-5
            loss_plus = loss(theta + delta, target, L)
            loss_minus = loss(theta - delta, target, L)
            grad[i] = (loss_plus - loss_minus) / (2 * 1e-5)
        
        # Update joint angles and clamp them within [-π, π]
        theta -= scaled_lr * grad
        theta = np.clip(theta, -np.pi, np.pi)
    
    return theta


# Check if a given position is inside the obstacle region 
def in_obstacle(pos, L, margin=0.0):
    obstacle_center = np.array([2 * L, 0.5 * L])
    obstacle_radius = L / 8
    return np.linalg.norm(pos - obstacle_center) < (obstacle_radius + margin)