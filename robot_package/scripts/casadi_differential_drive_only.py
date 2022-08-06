#!/usr/bin/python3
# from sys import path
# path.append(r"/home/prajwal/Libraries/Casadi_Py_Lib/")
import casadi
from casadi import *
import matplotlib.pyplot as plt
import numpy as np
import time


class Traj_Planner(object):
    def __init__(self, opti, prbls):
        self.opti = opti
        self.N = prbls['N']
        self.dt = prbls['dt']
        self.No_obs = prbls['No_obs']
        N = self.N
        x = self.opti.variable(1, N + 1)
        y = self.opti.variable(1, N + 1)
        theta = self.opti.variable(1, N + 1)
        self.X = vertcat(x, y, theta)
        v = self.opti.variable(1, N)
        w = self.opti.variable(1, N)
        self.U = vertcat(v, w)
        self.X_0 = self.opti.parameter(3, 1)
        self.X_f = self.opti.parameter(3, 1)
        self.X_dot = self.f_x_u()
        self.subject_to_dynamic_constraints()
        self.subject_to_static_constraints()
        self.Obs_pos = self.opti.parameter(2, self.No_obs)
        self.Obs_rad = self.opti.parameter(1, self.No_obs)
        cost_obstacles = self.Obstacle()
        self.opti.minimize(1 * sum2((x - self.X_f[0]) ** 2) + 1 * sum2((y - self.X_f[1]) ** 2) + 1 * sum2(
            (theta - self.X_f[2]) ** 2) + 0.5 * sum1(sum2((self.U) ** 2)) + cost_obstacles)
        p_opts = {"expand": True}
        s_opts = {"max_iter": 1000}
        opti.solver("ipopt", p_opts, s_opts)

    def return_solution(self, X_now, X_ref, Obstacles, Initialization=None):
        if not (Initialization is None):
            self.opti.set_initial(self.U, Initialization['U'])
            self.opti.set_initial(self.X, Initialization['X'])
        if self.No_obs > 0:
            self.opti.set_value(self.Obs_pos, Obstacles[0:2, 0:self.No_obs])
            self.opti.set_value(self.Obs_rad, Obstacles[2, 0:self.No_obs])
        self.opti.set_value(self.X_0, X_now)
        self.opti.set_value(self.X_f, X_ref)
        sol = self.opti.solve()
        return sol

    def f_x_u(self):
        x = MX.sym('x', 3)
        u = MX.sym('u', 2)
        f_x_u = vertcat(u[0] * cos(x[2]), u[0] * sin(x[2]), u[1])
        X_dot_act = Function('X_dot', [x, u], [f_x_u])
        return X_dot_act

    def subject_to_static_constraints(self):
        self.opti.subject_to(self.X[:, 0] == self.X_0)
        self.opti.subject_to(0 <= self.X[:])
        self.opti.subject_to(self.X[:] <= 12)
        self.opti.subject_to(self.U[0, :] <= 1)
        self.opti.subject_to(-1 <= self.U[0, :])
        self.opti.subject_to(self.U[1, :] <= 1.5)
        self.opti.subject_to(-1.5 <= self.U[1, :])

    def subject_to_dynamic_constraints(self):
        for k in range(self.N):
            self.opti.subject_to(self.X[:, k + 1] == self.X[:, k] + self.dt * self.X_dot(self.X[:, k], self.U[:, k]))

    def Obstacle(self):
        obst_cost = 0
        if self.No_obs > 0:
            No_obs = self.No_obs
            Obs_pos = self.Obs_pos
            Obs_rad = self.Obs_rad + 0.1
            for i in range(No_obs):
                h = log(((self.X[0, :] - Obs_pos[0, i]) / Obs_rad[i]) ** 2 + (
                        (self.X[1, :] - Obs_pos[1, i]) / Obs_rad[i]) ** 2)
                self.opti.subject_to(h > 0)
                obst_cost = obst_cost + sum2(exp(5 * exp(-h)))
        return obst_cost


def main():
    opti = casadi.Opti()
    dt = 0.2
    prbls = {'N': 10, 'dt': dt, 'No_obs': 5}
    simtime = 20
    traj = Traj_Planner(opti, prbls)
    print(traj.opti)
    X_now = [0, 0, 0]
    X_ref = [10, 10, pi]
    Obstacles = horzcat([8, 4, 0.5], [6, 6, 0.5], [4, 8, 0.5], [8.5, 6.5, 0.5], [6.5, 8.5, 0.5])
    # MPC loop:
    iter = int(simtime / prbls['dt'])
    X_es = horzcat(X_now, DM(3, iter))
    U_s = DM(2, iter)
    sol = traj.return_solution(X_now, X_ref, Obstacles)
    Initialization = {'X': sol.value(traj.X), 'U': sol.value(traj.U)}
    tic = time.time()
    future_traj = []
    for i in range(iter):
        sol = traj.return_solution(X_now, X_ref, Obstacles, Initialization)
        u_set = sol.value(traj.U)
        x_set = sol.value(traj.X)
        Initialization = {'X': x_set, 'U': u_set}
        # X_now = DM(X_now) + dt*traj.X_dot(X_now,u_set[:,0]).full()
        X_now = x_set[:, 1]
        future_traj.append(x_set)
        X_es[:, i + 1] = X_now
        U_s[:, i] = u_set[:, 0]
    toc = time.time()
    print(toc - tic, 'seconds elapsed')
    x = X_es[0, :]
    y = X_es[1, :]
    theta = X_es[2, :]
    v = U_s[0, :]
    omega = U_s[1, :]
    t = [i * dt for i in range(iter + 1)]
    plt.plot(x.T, y.T)
    for i in range(prbls['No_obs']):
        angl = [k / 100 * 2 * pi for k in range(101)]
        plt.plot(Obstacles[0, i] + Obstacles[2, i] * cos(angl), Obstacles[1, i] + Obstacles[2, i] * sin(angl))
    plt.axis('square')
    plt.title('x-y trajectory')
    plt.savefig('figure_traj.png')
    plt.show()
    plt.plot(t, x.T, label="x coordinate")
    plt.plot(t, y.T, label="y coordinate")
    plt.plot(t, theta.T, label="heading angle")
    plt.plot([0, 20], [10, 10], '--')
    plt.plot([0, 20], [pi, pi], '--')
    plt.legend(loc="upper left")
    plt.savefig('xandyandtheta.png')
    plt.show()
    plt.step(t, np.append(v, v[-1]), label="velocity (m/s)")
    plt.step(t, np.append(omega, omega[-1]), label="angular velocity (rad/s)")
    plt.legend(loc="upper right")
    plt.savefig('vandomega.png')
    plt.show()
    # plot_diffdrive_control(v, omega, t)
    # showprogression(x.T,y.T, theta.T, prbls, Obstacles, future_traj)


def plot_diffdrive_control(v, omega, t):
    L = 0.15  # 15 cm
    r = 0.05  # 5 cm
    omega_r = (2 * v + omega * L) / 2 / r
    omega_l = (2 * v - omega * L) / 2 / r
    plt.step(t, np.append(omega_l, omega_l[-1]), label="angular velocity of left wheel (rad/s)")
    plt.step(t, np.append(omega_r, omega_r[-1]), label="angular velocity of right wheel (rad/s)")
    plt.legend(loc="upper right")
    plt.savefig('omega_landr.png')
    plt.show()


def showprogression(x, y, theta, prbls, Obstacles, future_traj):
    L = 0.5
    H = 0.625
    interval = len(x.full()) // 5
    for i in range(0, len(x.full()) + 1, interval):
        tri = givemexandy((x[i], y[i]), L, H, theta[i])
        plt.plot(x[0:i + 1], y[0:i + 1])
        for j in range(prbls['No_obs']):
            angl = [k / 100 * 2 * pi for k in range(101)]
            plt.plot(Obstacles[0, j] + Obstacles[2, j] * cos(angl), Obstacles[1, j] + Obstacles[2, j] * sin(angl))
        plt.fill(tri[0, :], tri[1, :], "m")
        if i < len(future_traj):
            curr_fut = future_traj[i]
            plt.plot(curr_fut[0, :], curr_fut[1, :])
        plt.axis('square')
        plt.savefig('figure' + str(i * prbls['dt']) + '.png')
        plt.show()


def givemexandy(center_location, L, H, phii):
    center0 = center_location[0]
    center1 = center_location[1]
    R = np.array([[np.sin(phii), np.cos(phii)], [-np.cos(phii), np.sin(phii)]])
    X = ([-L / 2, L / 2, L / 2, -L / 2])
    Y = ([-H / 2, -H / 2, H / 2, H / 2])
    T = np.zeros([2, 4])
    for i in range(4):
        T[:, i] = np.dot(R, np.array([[X[i], Y[i]]]).T).reshape(2)
    x_lower_left = center0 + T[0, 0]
    x_lower_right = center0 + T[0, 1]
    x_upper_right = center0 + T[0, 2]
    x_upper_left = center0 + T[0, 3]
    x_upper = (x_upper_left + x_upper_right) / 2
    y_lower_left = center1 + T[1, 0]
    y_lower_right = center1 + T[1, 1]
    y_upper_right = center1 + T[1, 2]
    y_upper_left = center1 + T[1, 3]
    y_upper = (y_upper_left + y_upper_right) / 2
    x_coor = [x_lower_left, x_lower_right, x_upper]
    y_coor = [y_lower_left, y_lower_right, y_upper]
    xandy = np.array([x_coor, y_coor])
    return xandy


if __name__ == '__main__':
    main()
