import numpy as np
import matplotlib.pyplot as plt
from qpsolvers import solve_qp

def dynamic(s, u, dt):
    v = u[0]
    omega = u[1]
    delta = u[2]
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

def qp_solver(s, sd, so, alpha, beta, d, k_f, k_h, k_delta):
    # heading theta
    theta = s[2][0]

    # g matrix
    g = gs(theta)

    # xy position difference between current position and target position
    x_diff = sd[0][0] - s[0][0] 
    y_diff = sd[1][0] - s[1][0]
    xy_diff = np.array([[x_diff],
                        [y_diff]])
    
    # xy position difference between current position and obstacle position
    xo_diff = so[0][0] - s[0][0]
    yo_diff = so[1][0] - s[1][0]
    xyo_diff = np.array([[xo_diff],
                         [yo_diff]])

    # angle difference between current positional theta and target position
    Pd_theta = np.arctan2(y_diff, x_diff)
    Pd_theta_diff = angle_diff(Pd_theta, theta)

    # angle difference between current positional theta and obstacle position
    Po_theta = np.arctan2(yo_diff, xo_diff)
    Po_theta_diff = angle_diff(Po_theta+np.pi, theta)

    # l vector indicate relationship between current positoin and target position
    ld = ls(x_diff, y_diff)

    # l vector indicate relationship between current positoin and osbtacle position
    lo = ls(xo_diff, yo_diff)

    # minimize u and delta: [v omega delta] @ P @ [v omega delta].T + q.T @ [v omega delta].T = ||u||^2 + k_delta * delta^2
    # u = [v omega].T
    # x = [v omega delta].T
    # subject to Gx ≤ h, Ax = b, lb ≤ x ≤ ub

    num_osbtacle = so.shape[1]
    P = np.zeros((2+num_osbtacle, 2+num_osbtacle))
    P[:2,:2] = np.eye(2)
    for i in range (num_osbtacle):
        P[2+i][2+i] = k_delta
    q = np.zeros((2+num_osbtacle,1))

    G = np.zeros((1+num_osbtacle,2+num_osbtacle))

    # navigation constraints
    G[0][:2] = -2 * xy_diff.T @ g[:2] + 2 * k_f * Pd_theta_diff * ld.T @ g
    G[0][2] = 1

    # obstacle avoid constraints
    for i in range (num_osbtacle):
        G[1+i][:2] = 2 * xyo_diff.T @ g[:2] + 2 * k_h * Po_theta_diff * lo.T @ g
        G[1+i][2] = 0

    h = np.zeros((1+num_osbtacle,1))
    h[0][0] = -alpha * (x_diff**2 + y_diff**2 + k_f * (Pd_theta_diff)**2)
    for i in range (num_osbtacle):
        h[1+i][0] = beta * (xo_diff**2 + yo_diff**2 - d**2 - k_h * (Po_theta_diff)**2)

    x = solve_qp(P, q, G, h, solver="cvxopt")
    return x

def main(s, sd, so, alpha, beta, d, k_f, k_h, k_delta):
    x = qp_solver(s, sd, so, alpha, beta, d, k_f, k_h, k_delta)
    s = dynamic(s, x, dt)
    return s

# # state parameter
# s = np.array([[2.0], [3.0], [-np.pi/6]])  # state vector [Px, Py, theta]
# sd = np.array([[-4.0], [-2.0], [0.0]])  # target state [Px_d, Py_d, theta_d]

# # osbtacle parameter
# so = np.array([[-1.0], [-0.25], [0.0]]) # obstalce state [Px_o, Py_o, theta_o]

# state parameter
s = np.array([[0.0], [0.0], [0]])  # state vector [Px, Py, theta]
sd = np.array([[0.75], [0.75], [np.pi/2]])  # target state [Px_d, Py_d, theta_d]

# osbtacle parameter
so = np.array([[0.5], [0.5], [0.0]]) # obstalce state [Px_o, Py_o, theta_o]

# control parameter
alpha = 5 # higher when increase velocity (Proportional controller)
beta = 100 # higher when avoid the obstacle aggressively
k_f = 4 # higher when difficult to pointing to the target, lower when damping occur frequently
k_h = 0.5 # 0 means stop at the boundary, higher when not enough in avoid angle, lower when avoid angle too much
k_delta = 1 # gain of slack variable, higher when follow the constraints more strictly, lower when no need to follow the constraint too strictly
d = 0.1 # obstacle radius

# simulation parameter
dt = 0.001
T = 30
num_steps = int(T/dt)
states = []

plt.scatter(s[0], s[1], color='green', label='Start')
plt.scatter(sd[0], sd[1], color='red', label='Target')
plt.scatter(so[0][0], so[1][0], color='blue', label='osbtacle')
circle = plt.Circle((so[0][0], so[1][0]), d, color='blue', fill=False)
plt.gca().add_patch(circle)

for i in range(num_steps):
    s = main(s, sd, so, alpha, beta, d, k_f, k_h, k_delta)
    states.append(s)

states = np.array(states)
# # Add yaw angle arrow representation for every 10 steps
# for i in range(0, len(states), 5):
#     plt.quiver(states[i, 0], states[i, 1], np.cos(states[i, 2]), np.sin(states[i, 2]), angles='xy', color="purple", scale_units='xy', scale=0.1, width=0.01)

plt.plot(states[:, 0], states[:, 1], color="black", label='Trajectory')
plt.xlabel('x')
plt.ylabel('y')
plt.legend()
plt.title('Car Trajectory')
plt.grid(True)
plt.show()