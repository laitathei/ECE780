import numpy as np
import matplotlib.pyplot as plt

# Robot parameters (link lengths)
L1 = 1.0
L2 = 1.0
L3 = 1.0

# Desired end-effector position and orientation (x, y, theta)
desired_pose = np.array([2.0, 1.0, np.pi / 4])

# Initial joint angles (in radians)
theta1 = 0.0
theta2 = 0.0
theta3 = 0.0

# Learning rate
alpha = 0.1

# Tolerance for the position and orientation error
tolerance = 0.01

def forward_kinematics(theta1, theta2, theta3):
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2) + L3 * np.cos(theta1 + theta2 + theta3)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2) + L3 * np.sin(theta1 + theta2 + theta3)
    theta = theta1 + theta2 + theta3
    return np.array([x, y, theta])

def jacobian(theta1, theta2, theta3):
    J = np.zeros((3, 3))
    J[0, 0] = -L1 * np.sin(theta1) - L2 * np.sin(theta1 + theta2) - L3 * np.sin(theta1 + theta2 + theta3)
    J[0, 1] = -L2 * np.sin(theta1 + theta2) - L3 * np.sin(theta1 + theta2 + theta3)
    J[0, 2] = -L3 * np.sin(theta1 + theta2 + theta3)
    J[1, 0] = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2) + L3 * np.cos(theta1 + theta2 + theta3)
    J[1, 1] = L2 * np.cos(theta1 + theta2) + L3 * np.cos(theta1 + theta2 + theta3)
    J[1, 2] = L3 * np.cos(theta1 + theta2 + theta3)
    J[2, 0] = 1
    J[2, 1] = 1
    J[2, 2] = 1
    return J

def plot_robot(theta1, theta2, theta3):
    x0, y0 = 0, 0
    x1 = L1 * np.cos(theta1)
    y1 = L1 * np.sin(theta1)
    x2 = x1 + L2 * np.cos(theta1 + theta2)
    y2 = y1 + L2 * np.sin(theta1 + theta2)
    x3 = x2 + L3 * np.cos(theta1 + theta2 + theta3)
    y3 = y2 + L3 * np.sin(theta1 + theta2 + theta3)
    
    plt.plot([x0, x1], [y0, y1], 'b-', label='Link 1' if plt.gca().get_legend_handles_labels()[1] == [] else "")
    plt.plot([x1, x2], [y1, y2], 'g-', label='Link 2' if plt.gca().get_legend_handles_labels()[1] == [] else "")
    plt.plot([x2, x3], [y2, y3], 'm-', label='Link 3' if plt.gca().get_legend_handles_labels()[1] == [] else "")
    plt.plot([x0, x1, x2, x3], [y0, y1, y2, y3], 'ro')
    
    plt.xlim(-3, 3)
    plt.ylim(-3, 3)
    plt.plot(desired_pose[0], desired_pose[1], 'yo', label='Desired Position' if plt.gca().get_legend_handles_labels()[1] == [] else "")
    
    if plt.gca().get_legend_handles_labels()[1] == []:
        plt.legend()
    
    plt.draw()
    plt.pause(0.1)
    plt.clf()

# Iterative update using Jacobian Transpose method
for i in range(1000):
    current_pose = forward_kinematics(theta1, theta2, theta3)
    error = desired_pose - current_pose
    
    if np.linalg.norm(error) < tolerance:
        print(f"Converged in {i} iterations")
        break
    
    J = jacobian(theta1, theta2, theta3)
    dtheta = alpha * J.T @ error
    
    theta1 += dtheta[0]
    theta2 += dtheta[1]
    theta3 += dtheta[2]
    
    plot_robot(theta1, theta2, theta3)

plt.show()
