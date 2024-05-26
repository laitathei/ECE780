import numpy as np
import matplotlib.pyplot as plt

class numerical_IK:
    def __init__(self, link_length, joint_angle, target_position):
        self.link_length = link_length
        self.l1 = self.link_length[0]
        self.l2 = self.link_length[1]
        self.l3 = self.link_length[2]
        self.theta = joint_angle  # Initial joint angles
        self.theta1 = self.theta[0]
        self.theta2 = self.theta[1]
        self.theta3 = self.theta[2]
        self.target_position = target_position

    def FK(self):
        x = self.l1*np.cos(self.theta1)+self.l2*np.cos(self.theta1+self.theta2)+self.l3*np.cos(self.theta1+self.theta2+self.theta3)
        y = self.l1*np.sin(self.theta1)+self.l2*np.sin(self.theta1+self.theta2)+self.l3*np.sin(self.theta1+self.theta2+self.theta3)
        theta = self.theta1 + self.theta2 + self.theta3
        return np.array([x, y, theta])

    def J(self):
        matrix = np.zeros((3,3))
        matrix[0][0] = -self.l1*np.sin(self.theta1)-self.l2*np.sin(self.theta1+self.theta2)-self.l3*np.sin(self.theta1+self.theta2+self.theta3)
        matrix[0][1] = -self.l2*np.sin(self.theta1+self.theta2)-self.l3*np.sin(self.theta1+self.theta2+self.theta3)
        matrix[0][2] = -self.l3*np.sin(self.theta1+self.theta2+self.theta3)
        matrix[1][0] = self.l1*np.cos(self.theta1)+self.l2*np.cos(self.theta1+self.theta2)+self.l3*np.cos(self.theta1+self.theta2+self.theta3)
        matrix[1][1] = self.l2*np.cos(self.theta1+self.theta2)+self.l3*np.cos(self.theta1+self.theta2+self.theta3)
        matrix[1][2] = self.l3*np.cos(self.theta1+self.theta2+self.theta3)
        matrix[2][0] = 1
        matrix[2][1] = 1
        matrix[2][2] = 1
        return matrix

    def IK(self, max_iteration=1000, gamma=0.5, epsilon=1e-3):
        current_iteration = 0
        error = float('inf')
        self.trajectory = []
        while (current_iteration < max_iteration) and (error > epsilon):
            # Calculate current position by current theta angle
            current_position = self.FK()

            # Calculate the error between current position and target position
            error = self.target_position - current_position

            # Break while loop when the error smaller than the minimum error requirement
            if np.linalg.norm(error) < epsilon:
                break

            # Append current position for display End-Effector trajectory
            self.trajectory.append(current_position[:2])

            # Compute the Jacobian Transpose
            # J = self.J()
            # d_theta = gamma * J.T @ error

            # Compute the Jacobian Pseudo-Inverse
            J = self.J()
            J = np.linalg.pinv(J) # both right and left pseudo inverse solved by SVD
            d_theta = gamma * J @ error

            # Update theta
            self.theta += d_theta
            self.theta1 = self.theta[0]
            self.theta2 = self.theta[1]
            self.theta3 = self.theta[2]

            # Update iteration
            current_iteration += 1
            error = np.linalg.norm(error)

        self.trajectory = np.array(self.trajectory)
        self.plot_trajectory()
        print("IK calculation finish")
        print("Iteration time: ", current_iteration)
        print("Link Length: ", self.link_length)
        print("Target Position: ", target_position)
        print("Joint Angles (Degrees): ", self.theta*180/np.pi)
        print("Final Calculated End Effector Position: ", self.FK())

    def plot_trajectory(self):
        # Plot the trajectories of the end-effector
        plt.figure()
        plt.plot(self.trajectory[:, 0], self.trajectory[:, 1], 'o-', label='End Effector Trajectory')
        plt.plot(self.target_position[0], self.target_position[1], 'rx', label='Target Pose')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.title('End Effector Trajectory')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()

link_length = np.array([1.0, 1.0, 1.0], dtype=float)
joint_angle = np.array([0, 0, 0], dtype=float)
target_position = np.array([2.5, 0.5, np.pi/4], dtype=float)
RRR_manipulator = numerical_IK(link_length, joint_angle, target_position)
RRR_manipulator.IK()
