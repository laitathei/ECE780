import numpy as np
import matplotlib.pyplot as plt
from qpsolvers import solve_qp

def dynamic(s, u, dt):
    v = u[0]
    omega = u[1]
    Px, Py, theta = s[0][0], s[1][0], s[2][0]
    Px = Px + v*np.cos(theta)* dt
    Py = Py + v*np.sin(theta) * dt
    theta = theta + omega * dt
    s = np.array([[Px],
                  [Py],
                  [theta]])
    return s

def angle_diff(theta_d, theta):
    # Elimiate the effect of dtheta over +/- 180
    dtheta = theta_d - theta
    if dtheta >= np.pi:
        dtheta = -2*np.pi+dtheta
    elif dtheta <= -np.pi:
        dtheta = 2*np.pi+dtheta
    return dtheta

def gs(theta):
    s = np.array([[np.cos(theta), 0],
                  [np.sin(theta), 0],
                  [0, 1]])
    return s

def ls(x_diff, y_diff):
    d = x_diff**2 + y_diff**2 # Euclidean distance
    l = np.array([[y_diff/d],
                  [-x_diff/d],
                  [-1]])
    return l

def qp_solver(s, sd, alpha, k_f):
    theta = s[2][0]
    g = gs(theta)
    x_diff = sd[0][0] - s[0][0]
    y_diff = sd[1][0] - s[1][0]
    xy_diff = np.array([[x_diff],
                        [y_diff]])
    Pd_theta = np.arctan2(y_diff, x_diff)
    Pd_theta_diff = angle_diff(Pd_theta, theta)
    ld = ls(x_diff, y_diff)

    # minimize u: u.T @ P @ u + q.T @ u = ||u||^2 
    # subject to Gu ≤ h, Au = b, lb ≤ u ≤ ub
    P = np.eye(2)
    q = np.zeros((2,1))
    G = -2 * xy_diff.T @ g[:2] + 2 * k_f * Pd_theta_diff * ld.T @ g
    h = np.array([-alpha * (x_diff**2 + y_diff**2 + k_f * (Pd_theta_diff)**2)])
    u = solve_qp(P, q, G, h, solver="cvxopt")
    return u

def main(s, sd, alpha, k_f, dt):
    u = qp_solver(s, sd, alpha, k_f)
    g = gs(s[2][0])
    s = dynamic(s, u, dt)
    return s

# state parameter
s = np.array([[0.0], [0.0], [0.0]])  # state vector [Px, Py, theta]
sd = np.array([[0.75], [0.75], [np.pi/2]])  # target state [Px_d, Py_d, theta_d]

# control parameter
alpha = 0.1
k_f = 0.1

# simulation parameter
dt = 0.1
T = 300
num_steps = int(T/dt)
states = []
plt.scatter(s[0], s[1], color='green', label='Start')
plt.scatter(sd[0], sd[1], color='red', label='Target')

for i in range(num_steps):
    s = main(s, sd, alpha, k_f, dt)
    states.append(s)

states = np.array(states)
plt.plot(states[:, 0], states[:, 1], label='Trajectory')
plt.xlabel('x')
plt.ylabel('y')
plt.legend()
plt.title('Car Trajectory')
plt.grid(True)
plt.show()