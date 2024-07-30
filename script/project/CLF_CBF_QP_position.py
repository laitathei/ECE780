import numpy as np
import matplotlib.pyplot as plt
from control import qp_solver, Vs, Hs, plot_graph

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
    delta = x[2]
    s = dynamic(s, x, dt)
    return s, delta

# state parameter
s = np.array([[2.0], [3.0], [-np.pi/6]])  # state vector [Px, Py, theta]
sd = np.array([[-4.0], [-2.0], [0.0]])  # target state [Px_d, Py_d, theta_d]

# osbtacle parameter
so = np.array([[-1.0, 1.0], [-0.25, -4.0], [0.0, 0.0]]) # obstalce state [Px_o, Py_o, theta_o]

# control parameter
alpha = 5
beta = 20
k_f = 8
k_h = 4
k_delta = 15
d = 1.5 # obstacle radius

# simulation parameter
dt = 0.001
T = 0.9
num_steps = int(T/dt)
states = []
vs_list = []
hs_list = []
delta_list = []
iterations = []
num_obstacles = so.shape[1]

plt.scatter(s[0], s[1], color='green', label='Start')
plt.scatter(sd[0], sd[1], color='red', label='Target')
for i in range (num_obstacles):
    plt.scatter(so[0][i], so[1][i], color='blue', label='osbtacle')
    circle = plt.Circle((so[0][i], so[1][i]), d, color='blue', fill=False)
    plt.gca().add_patch(circle)

for i in range(num_steps):
    s, delta = main(s, sd, so, alpha, beta, d, k_f, k_h, k_delta)
    vs_list.append(Vs(s, sd, k_f))
    hs_list.append(Hs(s, so, k_h, d))
    delta_list.append(delta)
    iterations.append(i)
    states.append(s)

print(s)
print(s.shape)
states = np.array(states)
plt.plot(states[:, 0], states[:, 1], color="black", label='Trajectory')
plt.xlabel('x')
plt.ylabel('y')
plt.legend()
plt.title('Car Trajectory')
plt.grid(True)

plot_graph(vs_list, hs_list, delta_list, iterations)
# # Add yaw angle arrow representation for every 10 steps
# for i in range(0, len(states), 5):
#     plt.quiver(states[i, 0], states[i, 1], np.cos(states[i, 2]), np.sin(states[i, 2]), angles='xy', color="purple", scale_units='xy', scale=0.1, width=0.01)
