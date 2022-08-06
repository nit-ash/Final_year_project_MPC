from math import *
from re import X
from time import sleep
import casadi
from casadi import *
import numpy as np
import rospy
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from robot_package.msg import Obstacle_pos, Control_signals
from tf.transformations import euler_from_quaternion
import serial
from parameters import *


# Defining Path planner class
class path_planner(object):

    def __init__(self, opti, params, x0, xs, max_iter, obs_no):
        self.opti = opti  # opti object of casadi
        self.max_iter = max_iter  # max iterations to run
        self.obs_no = obs_no  # no of obstacles to take into account
        self.N = params['N']  # Planning horizon
        self.dt = params['dt']
        self.obs_no = params['obs_no']
        N = self.N
        self.x0 = x0
        self.xs = xs

        # Define state variables
        x = self.opti.variable()
        y = self.opti.variable()
        theta = self.opti.variable()
        self.states = vertcat(x, y, theta)
        self.n_states = 3

        # Define control variables
        v = self.opti.variable()
        w = self.opti.variable()
        self.controls = vertcat(v, w)
        self.n_controls = 2

        self.rhs = self.system_equations(v, theta, w)  # system equation

        # Define mapping function
        self.f = Function('f', [self.states, self.controls], [self.rhs])

        # Define casadi parameters
        self.X = self.opti.variable(self.n_states, N + 1)
        self.U = self.opti.variable(self.n_controls, N)
        self.P = self.opti.parameter(2 * self.n_states)  # States Parameter
        self.obstacle_X = self.opti.parameter(self.obs_no)  # Obstacle X parameter
        self.obstacle_Y = self.opti.parameter(self.obs_no)  # Obstacle Y parameter
        self.obstacle_R = self.opti.parameter(self.obs_no)  # Obstacle radius parameter

        # Implement system dynamics symbolically
        self.dynamic_constraints()
        self.objective()  # Defining objective
        self.static_constraints()  # Defining static constraints
        self.obstacle_constraints()  # Defining obstacle constraints

        # Define solver
        self.opti.solver('ipopt')  # Interior Point Solver

        self.count = 0
        self.xpos = np.array((0,))
        self.ypos = np.array((0,))
        self.tpos = np.array((0,))

        self.xprec = np.zeros(self.N + 1)
        self.yprec = np.zeros(self.N + 1)
        self.tprec = np.zeros(self.N + 1)

    def obstacle_constraints(self):
        for i in range(self.obs_no):
            for k in range(self.N + 1):
                self.opti.subject_to(
                    sqrt((self.X[0, k] - self.obstacle_X[i]) ** 2 + (self.X[1, k] - self.obstacle_Y[i]) ** 2) > (
                                self.obstacle_R + 1))

    def mpc_loop(self):
        ser = serial.Serial("/dev/ttyACM0", 9600, timeout=1)  # Serial object for sending data over serial
        pub = rospy.Publisher("control_signals", Control_signals, queue_size=10)
        controls = Control_signals()
        while np.linalg.norm(
                (self.xs - self.x0)) > 1e-2 and self.count < self.max_iter:  # Change the position vector length
            rospy.sleep(0.1)  # Stop ros operations in the current for 0.1 sec so that path planner can execute

            # Setting obstacle parameter values 
            if len(obs_x) >= self.obs_no:
                self.opti.set_value(self.obstacle_X, obs_x[0:self.obs_no - 1])
                self.opti.set_value(self.obstacle_Y, obs_y[0:self.obs_no - 1])
                self.opti.set_value(self.obstacle_R, obs_r[0:self.obs_no - 1])

            if len(obs_x) < self.obs_no:
                flag = np.ones(self.obs_no - len(obs_x)) * 100
                radius_flag = np.zeros(self.obs_no - len(obs_x))
                self.opti.set_value(self.obstacle_X, vertcat(obs_x, flag))
                self.opti.set_value(self.obstacle_Y, vertcat(obs_x, flag))
                self.opti.set_value(self.obstacle_R, vertcat(obs_r, radius_flag))

            self.u0 = np.zeros((self.n_controls, self.N))
            self.opti.set_value(self.P, vertcat(self.x0, self.xs))  # Setting Robot parameters
            self.opti.set_initial(self.U, self.u0)  # Initializing controls
            self.sol = self.opti.solve()  # Solve the NLP
            self.u = self.sol.value(self.U)
            self.xdynamics = self.sol.value(X)

            # Calculating individual wheels omega from v and w
            self.omega1 = (2 * self.u[0] + self.u[1] * 0.6) / 0.4064  # Left wheel omega
            self.omega2 = (2 * self.u[0] - self.u[1] * 0.6) / 0.4064  # Right wheel omega

            # self.cas=str(self.omega1)+","+str(self.omega2)+" "  # Converting omega signals to string
            # ser.write(self.cas.encode("utf-8"))                 # Sending data over serial
            controls.wl = self.omega1
            controls.wr = self.omega2

            pub.publish(controls)

            self.x0 = vertcat(x, y, t)  # Updating robot location

            # Logging data for visualization
            self.xpos = np.append(self.xpos, self.x0[0])
            self.ypos = np.append(self.ypos, self.x0[1])
            self.tpos = np.append(self.tpos, self.x0[2])
            # print(xprec)
            self.xprec = np.vstack([self.xprec, self.xdynamics[0, :]])
            self.yprec = np.vstack([self.yprec, self.xdynamics[1, :]])
            self.tprec = np.vstack([self.tprec, self.xdynamics[2, :]])
            if self.count == 0:
                self.xprec = np.delete(self.xprec, 0, 0)
                self.yprec = np.delete(self.yprec, 0, 0)
                self.tprec = np.delete(self.tprec, 0, 0)

            self.count = self.count + 1

        return (self.xpos, self.ypos, self.tpos, self.xprec, self.yprec, self.tprec)

    def static_constraints(self):
        vel = self.U[0, :]
        omega = self.U[1, :]

        self.opti.subject_to(vel >= -max_vel)
        self.opti.subject_to(vel <= max_vel)
        self.opti.subject_to(omega <= max_omega)
        self.opti.subject_to(omega >= -max_omega)

    def objective(self):
        self.Q = np.zeros((self.n_states, self.n_states), dtype=float)

        for k in range(self.n_states):
            self.Q[k, k] = 0.01  # x y and theta have equal weights

        self.R = np.zeros((self.n_controls, self.n_controls), dtype=float)
        for k in range(self.n_controls):
            self.R[k, k] = 0.01

        self.obj = 0
        for k in range(self.N):
            st = self.X[:, k]
            con = self.U[:, k]
            self.obj = self.obj + mtimes(mtimes((st - self.P[3:6]).T, self.Q), (st - self.P[3:6])) + mtimes(
                mtimes((con).T, self.R), con)

        self.opti.minimize(self.obj)

    def dynamic_constraints(self):
        # States Integration
        self.X[:, 0] = self.P[0:self.n_states]  # Defining initial state
        # Euler integration
        for k in range(self.N):
            st = self.X[:, k]
            con = self.U[:, k]
            f_value = self.f(st, con)
            st_next = st + self.dt * f_value
            self.opti.subject_to(self.X[:, k + 1] == st_next)  # Implementing multiple shooting

    def system_equations(self, v, theta, w):
        self.rhs = vertcat(v * cos(theta), v * sin(theta), w)
        rhs = self.rhs
        return (rhs)


def odom_callback(data):
    global x
    global y
    global t

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    [a, b, t] = euler_from_quaternion(
        [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,
         data.pose.pose.orientation.w])


def obstacle_callback(obs_data):
    global obs_x
    global obs_y
    global obs_r

    obs_x = np.ones(obs_no)
    obs_y = np.ones(obs_no)
    obs_r = np.ones(obs_no)

    for k in range(obs_no):
        obs_x[k] = obs_data.locx[k]
        obs_y[k] = obs_data.locy[k]
        obs_r[k] = obs_data.radius[k]


def main():
    global obs_no
    opti = casadi.Opti()  # casadi opti object
    # dt=0.2                      # simulation step time
    # N=20                        # Planning Horizon
    # obs_no=5                    # no of obstacles
    parameters = {'N': N, 'dt': dt, 'obs_no': obs_no}  # Defining parameter dictionary object
    rospy.init_node('path_planner_node', anonymous=False)  # initializing path_planner_node
    rospy.Subscriber("/odom", Odometry, odom_callback)  # odometry subscriber
    rospy.Subscriber("/obstacles_location", Obstacle_pos, obstacle_callback)  # Obstacle loation subscriber
    sleep(2)  # Need ammendation
    rospy.sleep(0.1)
    # initial_position=vertcat(x,y,t)         # Define Robot initial Position
    # final_position=vertcat(10,10,pi)        # Define Robot Final Position
    max_iterations = 500  # max iterations in mpc loop
    traj = path_planner(opti, parameters, initial_position, final_position, max_iterations,
                        obs_no)  # Defining object to path planner class
    [xpos, ypos, tpos, xprec, yprec, tprec] = traj.mpc_loop()  # calling mpc_loop function

    # Visualization
    scale = 1  # Triangle Scale
    plt.figure()
    for k in range(len(xpos)):
        x_init = ([xpos[k] + 0.5 * scale * cos(tpos[k])], [xpos[k] + 0.3 * scale * cos(tpos[k] + 2 * pi / 3)],
                  [xpos[k] + 0.3 * scale * cos(tpos[k] - 2 * pi / 3)], [xpos[k] + 0.5 * scale * cos(tpos[k])])
        y_init = ([ypos[k] + 0.5 * scale * sin(tpos[k])], [ypos[k] + 0.3 * scale * sin(tpos[k] + 2 * pi / 3)],
                  [ypos[k] + 0.3 * scale * sin(tpos[k] - 2 * pi / 3)], [ypos[k] + 0.5 * scale * sin(tpos[k])])
        x_init = np.array(x_init)
        y_init = np.array(y_init)
        plt.clf()
        plt.plot(x_init, y_init, color='red')
        plt.fill(x_init, y_init, 'black')
        plt.plot(xpos[0:k], ypos[0:k], color='green')
        try:
            plt.plot(xprec[k, :], yprec[k, :], '-o', color='red')
        except:
            plt.plot(xprec[k - 1, :], yprec[k - 1, :], '-o', color='red')
        plt.xlim(0, 10)
        plt.ylim(0, 10)
        plt.pause(0.1)
    plt.show()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
