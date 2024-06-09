import numpy as np
import matplotlib.pyplot as plt

class manipulator_control:
    def __init__(self, L, ML, IL, D, qd, qd_dot, qd_dot_dot, q, q_dot, q_dot_dot):
        # RR manipulator situation
        self.L = L # length of each link
        self.ML = ML # mass of each link
        self.D = D # distance between link center of mass and its origin
        self.IL = IL # inertia of each link
        self.qd = qd # desired joint anlge
        self.qd_dot = qd_dot # desired joint velocity
        self.qd_dot_dot = qd_dot_dot # desired joint acceleration
        self.q = q # initial joint anlge
        self.q_dot = q_dot # initial joint velocity
        self.q_dot_dot = q_dot_dot # initial joint acceleration
        self.g = 9.81 # gravity
        self.Fc11 = 0
        self.Fc22 = 0
        self.Fv11 = 0.1
        self.Fv22 = 0.1
        self.dt = 0.01 # delta time
        self.T = 20.0 # simulation time
        self.omega = 5 # natural frequency
        self.zeta = 0.5 # damping ratio

    def get_dynamics(self, q, q_dot):
        """
        Calculate the dynamic of the manipulator based on the joint anlge and joint velocity

        :param np.ndarray q: joint angle
        :param np.ndarray q_dot: joint velocity
        :returns: 
            - D (np.ndarray) - Inertia matrix
            - C (np.ndarray) - Centrifugal and Coriolis matrix
            - F (np.ndarray) - Friction matrix
            - G (np.ndarray) - Gravity compensation
        """
        ML1, ML2 = self.ML[0][0], self.ML[1][0] # mass of each link
        D1, D2 = self.D[0][0], self.D[1][0] # distance between link center of mass and its origin
        L1, L2 = self.L[0][0], self.L[1][0] # length of each link
        IL1, IL2 = self.IL[0][0], self.IL[1][0] # inertia of each link
        q1, q2 = q[0][0], q[1][0] # joint angle
        q1_dot, q2_dot = q_dot[0][0], q_dot[1][0] # joint velocity

        # Jvc1 = np.array([[-D1*np.sin(q1), 0],
        #                  [D1*np.cos(q1), 0],
        #                  [0, 0]])
        # vc1 = Jvc1 @ q_dot
        # Jvc2 = np.array([[-D1*np.sin(q1)-D2*np.sin(q1+q2), -D2*np.sin(q1+q2)],
        #                  [D1*np.cos(q1)+D2*np.cos(q1+q2), D2*np.cos(q1+q2)],
        #                  [0, 0]])
        # vc2 = Jvc2 @ q_dot

        # Jw1 = np.array([[0, 0],
        #                 [0, 0],
        #                 [1, 0]])
        # w1 = Jw1 @ q_dot
        # Jw2 = np.array([[0, 0],
        #                 [0, 0],
        #                 [1, 1]])
        # w2 = Jw2 @ q_dot

        # Inertia Matrix
        # K1 (linear) = 0.5*ML1*vc1.T@vc1 + 0.5*ML2*vc2.T@vc2
        # K1 (linear) = 0.5*ML1*Jvc1.T@q_dot.T@Jvc1@q_dot + 0.5*ML2*Jvc2.T@q_dot.T@Jvc2@q_dot
        # K1 (linear) = 0.5*q_dot.T@(ML1*Jvc1.T@Jvc1 + ML2*Jvc2.T@Jvc2)@q_dot
        # K2 (rotation) = 0.5*IL1*w1.T@w1 + 0.5*IL2*w2.T@w2
        # K2 (rotation) = 0.5*IL1*Jw1.T@q_dot.T@Jw1@q_dot + 0.5*IL2*Jw2.T@q_dot.T@Jw2@q_dot
        # K2 (rotation) = 0.5*q_dot.T@(IL1*Jw1.T@Jw1 + IL1*Jw2.T@Jw2)@q_dot
        # K2 (rotation) = 0.5*q_dot.T@(np.array([[I1, 0],[0, 0]]) + np.array([[I2, I2],[I2, I2]]))@q_dot
        # K2 (rotation) = 0.5*q_dot.T@(np.array([[I1+I2, I2], [I2, I2]]))@q_dot
        # K (total) = K1 + K2
        # K (total) = 0.5*q_dot.T@(ML1*Jvc1.T@Jvc1 + ML2*Jvc2.T@Jvc2)@q_dot + 0.5*q_dot.T@(np.array([[I1+I2, I2], [I2, I2]]))@q_dot
        # D = ML1*Jvc1.T@Jvc1 + ML2*Jvc2.T@Jvc2 + np.array([[I1+I2, I2], [I2, I2]])
        # K (total) = 0.5*q_dot.T@D@q_dot
        d11 = ML1*pow(D1,2)+ML2*(pow(L1,2)+pow(D2,2)+2*L1*D2*np.cos(q2))+IL1+IL2
        d12 = ML2*(pow(D2,2)+L1*D2*np.cos(q2))+IL2
        d21 = ML2*(pow(D2,2)+L1*D2*np.cos(q2))+IL2
        d22 = ML2*(pow(D2,2))+IL2
        D = np.array([[d11, d12],[d21, d22]])

        # Centrifugal and Coriolis matrix
        # cijk = 0.5*(partial(dkj/qi)+partial(dki/qj)-partial(dij/qk))
        # c111 = 0.5*(partial(d11/q1)+partial(d11/q1)-partial(d11/q1)) = 0.5*(0+0-0) = 0
        # c112 = 0.5*(partial(d21/q1)+partial(d21/q1)-partial(d11/q2)) = 0.5*(0+0-(-2*ML2*L1*D2*np.sin(q2))) = ML2*L1*D2*np.sin(q2) = -h
        # c121 = 0.5*(partial(d12/q1)+partial(d11/q2)-partial(d12/q1)) = 0.5*(0+(-2*ML2*L1*D2*np.sin(q2))-0) = -ML2*L1*D2*np.sin(q2) = h
        # c122 = 0.5*(partial(d22/q1)+partial(d21/q2)-partial(d12/q2)) = 0.5*(0+(-ML2*L1*D2*np.sin(q2))-(-ML2*L1*D2*np.sin(q2))) = 0
        # c211 = 0.5*(partial(d11/q2)+partial(d12/q1)-partial(d21/q1)) = 0.5*((-2*ML2*L1*D2*np.sin(q2))+0-0) = -ML2*L1*D2*np.sin(q2) = h
        # c212 = 0.5*(partial(d21/q2)+partial(d22/q1)-partial(d21/q2)) = 0.5*((-ML2*L1*D2*np.sin(q2))+0-(-ML2*L1*D2*np.sin(q2))) = 0
        # c221 = 0.5*(partial(d12/q2)+partial(d12/q2)-partial(d22/q1)) = 0.5*((-ML2*L1*D2*np.sin(q2))+(-ML2*L1*D2*np.sin(q2))-0) = -ML2*L1*D2*np.sin(q2) = h
        # c222 = 0.5*(partial(d11/q1)+partial(d11/q1)-partial(d11/q1)) = 0.5*(0+0-0) = 0
        h = -ML2*L1*D2*np.sin(q2)
        # c111 = 0
        # c112 = -h
        # c121 = h
        # c122 = 0
        # c211 = h
        # c212 = 0
        # c221 = h
        # c222 = 0 

        # ckj = summation(cijk(q)qi_dot)
        # c11 = c111*q1_dot + c211*q2_dot = 0*q1_dot + h*q2_dot = h*q2_dot
        # c12 = c112*q1_dot + c212*q2_dot = h*q1_dot + h*q2_dot = h*q1_dot + h*q2_dot
        # c21 = c121*q1_dot + c221*q2_dot = -h*q1_dot + 0*q2_dot = -h*q1_dot
        # c22 = c122*q1_dot + c222*q2_dot = 0*q1_dot + 0*q2_dot = 0
        C = np.array([[h*q2_dot, h*q1_dot + h*q2_dot],
                      [-h*q1_dot, 0]])

        # Gravity component
        # P1 = ML1*g*D1*np.sin(q1)
        # P2 = ML2*g*(L1*np.sin(q1)+D2*np.sin(q1+q2))
        # P = P1 + P2
        g1 = (ML1*D1+ML2*L1)*self.g*np.cos(q1) + ML2*D2*self.g*np.cos(q1+q2) # partial derviative of (G/q1)
        g2 = ML2*D2*self.g*np.cos(q1 + q2) # partial derviative of (G/q2)
        G = np.array([[g1], [g2]])

        # Coulomb friction matrix
        # friction exist on object moving relative to each other
        Fc = np.diag(np.array([self.Fc11, self.Fc22]))

        # Viscous friction matrix
        # friction exist on fluid moving relative to each other
        Fv = np.diag(np.array([self.Fv11, self.Fv22]))

        return D, C, Fc, Fv, G

    def main(self, Kp, Ki, Kd, friction, inverse_dynamic, torque_bound, torque_limit):
        if inverse_dynamic == True:
            self.Kp = np.diag(np.array([pow(self.omega,2), pow(self.omega,2)])) # pow(omega,2)
            self.Ki = np.diag(np.array([Ki, Ki]))
            self.Kd = np.diag(np.array([2*self.zeta*self.omega, 2*self.zeta*self.omega])) # 2*zeta*omega
        else:
            self.Kp = np.diag(np.array([Kp, Kp])) # pow(omega,2)
            self.Ki = np.diag(np.array([Ki, Ki]))
            self.Kd = np.diag(np.array([Kd, Kd])) # 2*zeta*omega
        error = np.array([[0], [0]])
        q1_list = []
        q2_list = []
        tau1_list = []
        tau2_list = []
        iternation_list = []
        for i in range (int(self.T/self.dt)):
            # print()
            # Get dynamic by current joint angle and joint velocity
            D, C, Fc, Fv, G = self.get_dynamics(self.q, self.q_dot)

            # Acculmated error
            error = error + ((self.qd-self.q)*self.dt)

            if inverse_dynamic == True:
                # Inverse dynamic (Computed torque) aims to find non-linear feedback control law u to become zeros
                u = self.qd_dot_dot + self.Kp @ (self.qd-self.q) + self.Kd @ (self.qd_dot-self.q_dot) + self.Ki @ error

                # Calculate the torque
                if friction == True:
                    tau = D @ u + C @ self.q_dot + Fc @ np.sign(self.q_dot) + Fv @ self.q_dot + G
                else:
                    tau = D @ u + C @ self.q_dot + G
            else:
                # PID controller on torque
                tau = self.Kp @ (self.qd-self.q) + self.Kd @ (self.qd_dot-self.q_dot) + self.Ki @ error

            # Torque bound
            if torque_bound == True:
                tau[tau >= torque_limit] = torque_limit
                tau[tau <= -torque_limit] = -torque_limit

            # Calculate the joint acceleration by solving dynamic equation
            # Friction included or not
            if friction == True:
                self.q_dot_dot = np.linalg.inv(D) @ (tau - C @ self.q_dot - Fc @ np.sign(self.q_dot) - Fv @ self.q_dot - G)
            else:
                self.q_dot_dot = np.linalg.inv(D) @ (tau - C @ self.q_dot - G)

            # Update the joint angle and velocity
            self.q = self.q + self.q_dot * self.dt
            self.q_dot = self.q_dot + self.q_dot_dot * self.dt

            q1 = self.q[0][0]
            q2 = self.q[1][0]
            tau1 = tau[0][0]
            tau2 = tau[1][0]
            q1_list.append(q1)
            q2_list.append(q2)
            tau1_list.append(tau1)
            tau2_list.append(tau2)
            iternation_list.append(i)

        print("Target joint angle: ", self.qd[0][0], self.qd[1][0])
        print("Final joint angle: ", self.q[0][0], self.q[1][0])
        self.plot_robot(q1_list, q2_list, tau1_list, tau2_list, iternation_list)

    def plot_robot(self, q1_list, q2_list, tau1_list, tau2_list, iternation_list):
        plt.figure()
        # plot trajectory
        plt.subplot(2, 1, 1)
        plt.plot(iternation_list, q1_list, label='q1') # joint angle 1
        plt.plot(iternation_list, q2_list, label='q2') # joint angle 2
        plt.axhline(y=qd[0], color='r', linestyle='--', label='qd1') # joint velocity 1
        plt.axhline(y=qd[1], color='g', linestyle='--', label='qd2') # joint velocity 2
        plt.xlabel('Time [{}s]'.format(self.dt))
        plt.ylabel('Joint Angles [rad]')
        plt.legend()

        # plot torque
        plt.subplot(2, 1, 2)
        plt.plot(iternation_list, tau1_list, label='tau1') # joint torque 1
        plt.plot(iternation_list, tau2_list, label='tau2') # joint torque 2
        plt.xlabel('Time [{}s]'.format(self.dt))
        plt.ylabel('Joint Torques [Nm]')
        plt.legend()

        plt.tight_layout()
        plt.show()



L = np.array([[1.0], [0.5]], dtype=float) # length of each link
ML = np.array([[0.1], [0.05]], dtype=float) # mass of each link
D = np.array([[0.5], [0.25]], dtype=float) # distance between link center of mass and its origin
IL = np.array([[0.1], [0.05]], dtype=float) # inertia of each link
q = np.array([[0], [0]], dtype=float) # initial joint anlge
qd = np.array([[-2*np.pi/3], [2*np.pi/3]], dtype=float) # desired joint anlge
q_dot = np.array([[0], [0]], dtype=float) # initial joint velocity
qd_dot = np.array([[0], [0]], dtype=float) # desired joint velocity
q_dot_dot = np.array([[0], [0]], dtype=float) # initial joint acceleration
qd_dot_dot = np.array([[0], [0]], dtype=float) # desired joint acceleration
sim = manipulator_control(L, ML, IL, D, qd, qd_dot, qd_dot_dot, q, q_dot, q_dot_dot)
Kp = 6.0
Ki = 0.0
Kd = 2.0
friction = True
inverse_dynamic = True
torque_bound = True
torque_limit = 0.1 # 0.5 or 0.1
sim.main(Kp, Ki, Kd, friction, inverse_dynamic, torque_bound, torque_limit)

# Q1 PID controller, friction = False, inverse_dynamic = False, torque_bound = False
# Q2 Inverse dynamic controller, friction = True, inverse_dynamic = True, torque_bound = True