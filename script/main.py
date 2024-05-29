import numpy as np
import matplotlib.pyplot as plt

class numerical_IK:
    def __init__(self, link_length, joint_angle, target_position):
        self.link_length = link_length # Initial link
        self.l1 = self.link_length[0]
        self.l2 = self.link_length[1]
        self.l3 = self.link_length[2]
        self.theta = joint_angle  # Initial joint angles
        self.theta1 = self.theta[0]
        self.theta2 = self.theta[1]
        self.theta3 = self.theta[2]
        self.target_position = target_position # Initial target position
        self.max_length = self.l1 + self.l2 + self.l3

    def FK(self):
        # Calculate Forward Kinematics
        x = self.l1*np.cos(self.theta1)+self.l2*np.cos(self.theta1+self.theta2)+self.l3*np.cos(self.theta1+self.theta2+self.theta3)
        y = self.l1*np.sin(self.theta1)+self.l2*np.sin(self.theta1+self.theta2)+self.l3*np.sin(self.theta1+self.theta2+self.theta3)
        theta = self.theta1 + self.theta2 + self.theta3
        return np.array([x, y, theta])

    def J(self):
        # Calculate Jacobian Matrix
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
        # Calculate Inverse Kinematics
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

            # Compute the Jacobian Transpose
            # J = self.J()
            # J = J.T

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

            # Plot the movement
            self.plot_robot()

        print("IK calculation finish")
        print("Iteration time: ", current_iteration)
        print("Link Length: ", self.link_length)
        print("Target Position: ", target_position)
        print("Joint Angles (Degrees): ", self.theta*180/np.pi)
        print("Final Calculated End Effector Position: ", self.FK())

    def plot_robot(self):
        # Calculate each joint position
        x0, y0 = 0, 0
        x1 = self.l1 * np.cos(self.theta1)
        y1 = self.l1 * np.sin(self.theta1)
        x2 = x1 + self.l2 * np.cos(self.theta1 + self.theta2)
        y2 = y1 + self.l2 * np.sin(self.theta1 + self.theta2)
        x3 = x2 + self.l3 * np.cos(self.theta1 + self.theta2 + self.theta3)
        y3 = y2 + self.l3 * np.sin(self.theta1 + self.theta2 + self.theta3)
        
        plt.plot([x0, x1], [y0, y1], 'b-', label='Link 1' if plt.gca().get_legend_handles_labels()[1] == [] else "")
        plt.plot([x1, x2], [y1, y2], 'g-', label='Link 2' if plt.gca().get_legend_handles_labels()[1] == [] else "")
        plt.plot([x2, x3], [y2, y3], 'm-', label='Link 3' if plt.gca().get_legend_handles_labels()[1] == [] else "")
        plt.plot([x0, x1, x2, x3], [y0, y1, y2, y3], 'ro')
        
        plt.xlim(-self.max_length, self.max_length)
        plt.ylim(-self.max_length, self.max_length)
        plt.plot(self.target_position[0], self.target_position[1], 'yo', label='Desired Position' if plt.gca().get_legend_handles_labels()[1] == [] else "")
        
        if plt.gca().get_legend_handles_labels()[1] == []:
            plt.legend()
        
        plt.draw()
        plt.pause(0.01)
        plt.clf()

link_length = np.array([3.0, 4.0, 5.0], dtype=float)
joint_angle = np.array([0, 0, 0], dtype=float)
target_position = np.array([5, 7, np.pi/4], dtype=float)
RRR_manipulator = numerical_IK(link_length, joint_angle, target_position)
RRR_manipulator.IK()
