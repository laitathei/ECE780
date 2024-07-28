import numpy as np
import matplotlib.pyplot as plt
from control import qp_solver

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
s = np.array([[-1.0], [-1.2], [np.arctan2(1.2, 1)]])  # state vector [Px, Py, theta]
sd = np.array([[0.0], [0.0], [0.0]])  # target state [Px_d, Py_d, theta_d]

# osbtacle parameter
so = np.array([[-0.5], [-0.6], [0.0]]) # obstalce state [Px_o, Py_o, theta_o]

# # state parameter
# s = np.array([[0.0], [0.0], [0]])  # state vector [Px, Py, theta]
# sd = np.array([[0.75], [0.75], [np.pi/2]])  # target state [Px_d, Py_d, theta_d]

# # osbtacle parameter
# so = np.array([[0.5, -0.5], [0.5, -0.5], [0.0, 0.0]]) # obstalce state [Px_o, Py_o, theta_o]

# control parameter
alpha = 2 # higher when increase velocity (Proportional controller)
beta = 10 # higher when avoid the obstacle aggressively
k_f = 5 # higher when difficult to pointing to the target, lower when damping occur frequently
k_h = 1 # 0 means stop at the boundary, higher when not enough in avoid angle, lower when avoid angle too much
k_delta = 0.5 # gain of slack variable, higher when follow the constraints more strictly, lower when no need to follow the constraint too strictly
d = 0.3 # obstacle radius

# simulation parameter
dt = 0.001
T = 0.9
num_steps = int(T/dt)
states = []
num_obstacles = so.shape[1]

plt.scatter(s[0], s[1], color='green', label='Start')
plt.scatter(sd[0], sd[1], color='red', label='Target')
for i in range (num_obstacles):
    plt.scatter(so[0][i], so[1][i], color='blue', label='osbtacle')
    circle = plt.Circle((so[0][i], so[1][i]), d, color='blue', fill=False)
    plt.gca().add_patch(circle)

for i in range(num_steps):
    print(i)
    s = main(s, sd, so, alpha, beta, d, k_f, k_h, k_delta)
    states.append(s)
    print(s)

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