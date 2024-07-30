from mobile_manipulator_unicycle import MobileManipulatorUnicycle
from utils import action
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
from control import qp_solver, pickup_object, dropoff_object, stop, Vs, Hs

robot = action()
# control parameter
alpha = 3 # higher when increase velocity (Proportional controller)
beta = 8 # lower when avoid the obstacle aggressively
k_f = 0.5 # higher when difficult to pointing to the target, lower when damping occur frequently
k_h = 0.1 # 0 means stop at the boundary, higher when not enough in avoid angle, lower when avoid angle too much
k_delta = 3 # gain of slack variable, higher when follow the constraints more strictly, lower when no need to follow the constraint too strictly
d = 0.1 # obstacle radius
L = 0.49 # distance between current position and the pickup position
L2 = 0.5 # distance between current position and the dropoff position
reach_pickup = False
reach_dropoff = False
task_done = False
iteration = 0

# Initialize lists to store data
vs_list = []
hs_list = []
delta_lists = []
theta_list = []
trajectory_x = []
trajectory_y = []

# Set up the figure with GridSpec
fig = plt.figure(figsize=(20, 10))
gs = fig.add_gridspec(3, 2, width_ratios=[1, 2])

# Set up the line graphs on the left subplots
ax1 = fig.add_subplot(gs[0, 0])
ax1.set_xlabel('Iteration')
ax1.set_ylabel('Vs')
vs_line, = ax1.plot([], [], label='Vs', color='r')
ax1.legend()

ax2 = fig.add_subplot(gs[1, 0])
ax2.set_xlabel('Iteration')
ax2.set_ylabel('Hs')
hs_line, = ax2.plot([], [], label='Hs', color='g')
ax2.legend()

ax3 = fig.add_subplot(gs[2, 0])
ax3.set_xlabel('Iteration')
ax3.set_ylabel('Delta Values')
delta_lines = []
delta_colors = ['b', 'c', 'm']
for i in range(len(delta_colors)):
    line, = ax3.plot([], [], label=f'Delta{i+1}', color=delta_colors[i])
    delta_lines.append(line)
ax3.legend()

# Set up the trajectory plot on the right subplot
ax4 = fig.add_subplot(gs[:, 1])
ax4.set_title('Car Trajectory')
ax4.set_xlabel('X Position')
ax4.set_ylabel('Y Position')
trajectory_line, = ax4.plot([], [], label='Trajectory', color='k')
ax4.set_xlim(-5, 5)  # Set x-axis limits
ax4.set_ylim(-5, 5)  # Set y-axis limits
ax4.legend()

# Plot start, pickup, and dropoff points
start_pos = robot.get_pose()[0]
pickup_pos = robot.get_pose()[1]
dropoff_pos = robot.get_pose()[2]

ax4.scatter(start_pos[0], start_pos[1], color='purple', label='Start Point')
ax4.scatter(pickup_pos[0], pickup_pos[1], color='blue', label='Pickup Point')
ax4.scatter(dropoff_pos[0], dropoff_pos[1], color='green', label='Dropoff Point')

# Plot obstacles with circles indicating radius
obstacles = robot.get_pose()[3:]
obstacles = np.array(obstacles)
for obs in obstacles:
    circle = Circle((obs[0], obs[1]), d, color='red', fill=False, linestyle='dashed')
    ax4.add_patch(circle)
ax4.scatter(obstacles[:, 0], obstacles[:, 1], color='red', label='Obstacles')

while not task_done:
    # Get the robot's current pose.
    poses = robot.get_pose()

    while not (len(poses[0]) and len(poses[1]) and len(poses[2]) and len(poses[3]) and len(poses[4]) and len(poses[5])):
        poses = robot.get_pose()

    current_pos = np.array(poses[0])
    pickup = np.array(poses[1])
    dropoff = np.array(poses[2])
    obstacles = np.array(poses[3:])

    print("Pickup position: ", poses[1])
    print("Dropoff position: ", poses[2])
    print("Obstacles position: ", poses[3:])
    # pickup_object(robot)
    # exit()
    # dropoff_object(robot)
    # exit()
    
    Px, Py, theta = current_pos
    if reach_pickup == False and reach_dropoff == False:
        Px_d, Py_d, theta_d = pickup
    elif reach_pickup == True and reach_dropoff == False:
        Px_d, Py_d, theta_d = dropoff
    Px_o, Py_o, theta_o = obstacles[:, 0], obstacles[:, 1], obstacles[:, 2]

    s = np.array([[Px], [Py], [theta]]) # state vector [Px, Py, theta]
    sd = np.array([[Px_d], [Py_d], [theta_d]]) # target state [Px_d, Py_d, theta_d]
    so = np.array([Px_o, Py_o, theta_o]) # obstalce state [Px_o, Py_o, theta_o]
    print(np.linalg.norm(sd[:2]-s[:2]))
    u = qp_solver(s, sd, so, alpha, beta, d, k_f, k_h, k_delta)

    v = u[0]
    omega = u[1]
    delta_values = u[2:]
    print(so)
    print(u)
    vs = Vs(s, sd, k_f)
    hs = Hs(s, so, k_h, d)

    vs_list.append(vs)
    hs_list.append(hs)
    if len(delta_lists) == 0:
        delta_lists = [[] for _ in range(len(delta_values))]
    for i, delta in enumerate(delta_values):
        delta_lists[i].append(delta)
    trajectory_x.append(Px)
    trajectory_y.append(Py)
    theta_list.append(theta)

    if reach_pickup == False and reach_dropoff == False:
        if np.linalg.norm(sd[:2]-s[:2]) >= L:
            robot.set_mobile_base_speed_and_gripper_power(v=v, omega=omega, gripper_power=0.0, lapse=0.01)
        else:
            stop(robot)
            pickup_object(robot)
            reach_pickup = True
    elif reach_pickup == True and reach_dropoff == False:
        if np.linalg.norm(sd[:2]-s[:2]) >= L2:
            robot.set_mobile_base_speed_and_gripper_power(v=v, omega=omega, gripper_power=0.0, lapse=0.01)
        else:
            stop(robot)
            dropoff_object(robot)
            reach_dropoff = True
    else:
        task_done = True
        fig.savefig('script\\project\\picture\\record.png')
        print("Finish task")

    # Update line graphs
    vs_line.set_data(range(iteration + 1), vs_list)
    ax1.relim()
    ax1.autoscale_view()

    hs_line.set_data(range(iteration + 1), hs_list)
    ax2.relim()
    ax2.autoscale_view()

    for i, delta_line in enumerate(delta_lines):
        if i < len(delta_values):
            delta_line.set_data(range(iteration + 1), delta_lists[i])
    ax3.relim()
    ax3.autoscale_view()

    # Update trajectory plot
    trajectory_line.set_data(trajectory_x, trajectory_y)

    plt.pause(0.05)  # Pause for a short period to simulate real-time processing

    iteration += 1