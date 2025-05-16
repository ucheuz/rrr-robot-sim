import numpy as np

def target_trajectory(t, L):
    x = 2 * L
    y = 0.5 * L + 0.4 * L * np.sin(2 * np.pi * 1 * t)
    return np.array([x, y])