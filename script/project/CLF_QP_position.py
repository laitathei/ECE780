import numpy as np
import matplotlib.pyplot as plt

s = np.array([[0.0], [0.0], [0.0]])  # state vector [x, y, theta]
sd = np.array([[0.75], [0.75]])  # target state [x, y]
alpha = 0.2
dt = 0.1
T = 60
num_steps = int(T/dt)
states = []

def dynamic(s, u, dt):
    v = u[0][0]
    omega = u[1][0]
    x, y, theta = s[0][0], s[1][0], s[2][0]
    x = x + v*np.cos(theta)* dt
    y = y + v*np.sin(theta) * dt
    theta = theta + omega * dt
    s = np.array([[x],
                  [y],
                  [theta]])
    return s

def angle_diff(theta, theta_d):
    # Elimiate the effect of dtheta over +/- 180
    dtheta = theta - theta_d
    return np.arctan2(np.sin(dtheta), np.cos(dtheta))

def state_diff(s, sd):
    Px, Py, theta = s[0][0], s[1][0], s[2][0]
    Px_d, Py_d = sd[0][0], sd[1][0]
    x_diff = Px - Px_d
    y_diff = Py - Py_d
    theta_diff = angle_diff(np.arctan2(y_diff, x_diff), theta)
    s_diff = np.array([[x_diff],
                       [y_diff],
                       [theta_diff]])
    return s_diff

def g(theta):
    return np.array([[np.cos(theta),0], [np.sin(theta),0], [0,1]])

for i in range(num_steps):
    heading = s[2][0]
    s_diff = state_diff(s, sd)
    u_star = -alpha/2 * np.linalg.pinv(g(heading)) @ s_diff # optimal velocity in V and Omega
    s = dynamic(s, u_star, dt)
    states.append(s)

states = np.array(states)
plt.scatter(s[0], s[1], color='green', label='Start')
plt.scatter(sd[0], sd[1], color='red', label='Target')
plt.plot(states[:, 0], states[:, 1], label='Trajectory')
plt.xlabel('x')
plt.ylabel('y')
plt.legend()
plt.title('Car Trajectory')
plt.grid(True)
plt.show()