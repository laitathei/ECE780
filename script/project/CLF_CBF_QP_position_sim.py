from mobile_manipulator_unicycle_sim import MobileManipulatorUnicycleSim
import numpy as np
from matplotlib.patches import Circle
import matplotlib.pyplot as plt
from control import qp_solver, Vs, Hs

# Two options to initialize the simulator
# (i) random robot and object locations
robot = MobileManipulatorUnicycleSim(robot_id=1)
# (ii) specified robot and object locations
# robot = MobileManipulatorUnicycleSim(robot_id=1, robot_pose=[0, 0, 0.0], pickup_location=[0.75, 0.75, np.pi/2], dropoff_location=[-0.75, -0.75, 0], obstacles_location=[[0.375, 0.375, 0], [-0.375, -0.375, 0]])
# robot = MobileManipulatorUnicycleSim(robot_id=1, robot_pose=[2, 3, -np.pi/6], pickup_location=[-4, -2, 0], dropoff_location=[-0.75, -0.75, 0], obstacles_location=[[-1.0, -0.25, 0], [-0.5, -0.5, 0]])
# robot = MobileManipulatorUnicycleSim(robot_id=1, robot_pose=[-1.2414430716175224, -0.7432977937128682, 0.68839476162176], pickup_location=[2.053731649650307, -1.980779262494083, -0.3465721428126516], dropoff_location=[-0.38260115265110173, -1.8329390358202664, -1.3495335512140096], obstacles_location=[[1.1161265101471471, -1.5976014045687932, 0.23622601570985924], [0.7335087814135117, 1.1452518051442775, -0.02517160463368673]])

# control parameter
alpha = 3 # higher when increase velocity (Proportional controller)
beta = 5 # lower when avoid the obstacle aggressively
k_f = 2 # higher when difficult to pointing to the target, lower when damping occur frequently
k_h = 0.1 # 0 means stop at the boundary, higher when not enough in avoid angle, lower when avoid angle too much
k_delta = 1 # gain of slack variable, higher when follow the constraints more strictly, lower when no need to follow the constraint too strictly
d = 0.1 # obstacle radius
L = 0.4 # distance between current position and the target position
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

# Set up the figures
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))

# Set up the line graphs on the left subplot
ax1.set_title('Control Variables Over Iterations')
ax1.set_xlabel('Iteration')
ax1.set_ylabel('Values')
vs_line, = ax1.plot([], [], label='Vs', color='r')
hs_line, = ax1.plot([], [], label='Hs', color='g')
ax1.legend(loc='upper left')

# Secondary y-axis for delta values
ax3 = ax1.twinx()
ax3.set_ylabel('Delta Values')
delta_lines = []
delta_colors = ['b', 'c', 'm', 'y', 'k', 'orange', 'purple']
for i in range(len(delta_colors)):
    line, = ax3.plot([], [], label=f'Delta{i+1}', color=delta_colors[i])
    delta_lines.append(line)
# ax3.legend(loc='upper right')

# Set up the trajectory plot on the right subplot
ax2.set_title('Car Trajectory')
ax2.set_xlabel('X Position')
ax2.set_ylabel('Y Position')
trajectory_line, = ax2.plot([], [], label='Trajectory', color='k')
ax2.set_xlim(-5, 5)  # Set x-axis limits
ax2.set_ylim(-5, 5)  # Set y-axis limits
# ax2.legend()

# Plot start, pickup, and dropoff points
start_pos = robot.get_poses()[0]
pickup_pos = robot.get_poses()[1]
dropoff_pos = robot.get_poses()[2]

ax2.scatter(start_pos[0], start_pos[1], color='purple', label='Start Point')
ax2.scatter(pickup_pos[0], pickup_pos[1], color='blue', label='Pickup Point')
ax2.scatter(dropoff_pos[0], dropoff_pos[1], color='green', label='Dropoff Point')

# Plot obstacles with circles indicating radius
obstacles = robot.get_poses()[3]
obstacles = np.array(obstacles)
for obs in obstacles:
    circle = Circle((obs[0], obs[1]), d, color='red', fill=False, linestyle='dashed')
    ax2.add_patch(circle)
ax2.scatter(obstacles[:, 0], obstacles[:, 1], color='red', label='Obstacles')

while not task_done:
    # Get the robot's current pose.
    poses = robot.get_poses()
    print("")
    print("Current position: ", poses[0])
    if iteration !=0:
        print("Start position: ", trajectory_x[0], trajectory_y[0], theta_list[0])
    print("heading: ", np.degrees(np.arctan2(np.sin(poses[0][2]),np.cos(poses[0][2]))))
    print("Pickup position: ", poses[1])
    print("Dropoff position: ", poses[2])
    print("Obstacles position: ", poses[3])
    
    current_pos = poses[0]
    pickup = poses[1]
    dropoff = poses[2]
    obstacles = poses[3]

    current_pos = np.array(current_pos)
    pickup = np.array(pickup)
    dropoff = np.array(dropoff)
    obstacles = np.array(obstacles)

    Px, Py, theta = current_pos
    if reach_pickup == False and reach_dropoff == False:
        Px_d, Py_d, theta_d = pickup
    elif reach_pickup == True and reach_dropoff == False:
        Px_d, Py_d, theta_d = dropoff
    Px_o, Py_o, theta_o = obstacles[:, 0], obstacles[:, 1], obstacles[:, 2]

    s = np.array([[Px], [Py], [theta]]) # state vector [Px, Py, theta]
    sd = np.array([[Px_d], [Py_d], [theta_d]]) # target state [Px_d, Py_d, theta_d]
    so = np.array([Px_o, Py_o, theta_o]) # obstalce state [Px_o, Py_o, theta_o]

    u = qp_solver(s, sd, so, alpha, beta, d, k_f, k_h, k_delta)

    v = u[0]
    omega = u[1]
    delta_values = u[2:]
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
            robot.set_mobile_base_speed_and_gripper_power(v=v, omega=omega, gripper_power=0.0)
        else:
            reach_pickup = True
    elif reach_pickup == True and reach_dropoff == False:
        if np.linalg.norm(sd[:2]-s[:2]) >= L:
            robot.set_mobile_base_speed_and_gripper_power(v=v, omega=omega, gripper_power=0.0)
        else:
            reach_dropoff = True
    else:
        robot.set_mobile_base_speed_and_gripper_power(v=0, omega=0, gripper_power=0.0)
        task_done = True
        # Save the figures as PNG files
        fig.savefig('script\\project\\picture\\record.png')
        print("Finish task")

    # Update line graphs
    vs_line.set_data(range(iteration + 1), vs_list)
    hs_line.set_data(range(iteration + 1), hs_list)
    ax1.relim()
    ax1.autoscale_view()

    for i, delta_line in enumerate(delta_lines):
        if i < len(delta_values):
            delta_line.set_data(range(iteration + 1), delta_lists[i])
    ax3.relim()
    ax3.autoscale_view()

    # Update trajectory plot
    trajectory_line.set_data(trajectory_x, trajectory_y)

    plt.pause(0.05)  # Pause for a short period to simulate real-time processing

    iteration += 1