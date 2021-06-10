import numpy as np
import cvxpy as cp
from controller import Controller
from utils import Utils

class Robot:
    def __init__(self, N, ips=[]):
        assert N == len(ips), "ip list and N not match"
        self.N = N
        self.ips = ips

        self.pose = np.zeros([3, N])
        self.pose_time = [None] * N
        self.vel = np.zeros([3, N])
        self.prev_cmd_vel = np.zeros([2, N]) + 1e-4
        self.dts = None

        self.utils = Utils()
        self.ctl = Controller(N, vlim=0.05, wlim=0.9, vcf=1.5, wcf=0.1,
                        knob_length=10,
                        body_length=50,
                        padding_length=2.5)

        self.status = np.zeros(N)

    def update_prev_cmd_vel(self, data, i):
        try:
            vl, vr = data.split(',')
            vl, vr = [int(vl), int(vr)]
            self.prev_cmd_vel[:, i] = [vl, vr]
        except Exception as e:
            print e

    def fit_command_vel(self, gdu, th, cx=1.0, ct=1.0):
        N = self.N
        dx = np.zeros([3, N])
        vel = np.zeros([2, N])

        if not self.dts: return vel
        if np.linalg.norm(gdu) < th: return vel

        # parameters for hardware and optimizer
        (r, L, a_s, pcf, lcf) = (0.002, 0.28, 1e-3, 1e-6, 1.0)
        # min and max motor PWM values
        (v_min, v_max) = (57, 120)
        M = v_max + v_min

        v = cp.Variable(2*N)
        s = cp.Variable(2*N, integer=True)
        A = np.zeros([4*N, 2*N])
        b = np.zeros(4*N)

        for i in range(N):
            dt = self.dts[i]*3
            if not dt: continue

            A[4*i, (2*i):(2*i+2)] = dt * r/2 * (np.zeros(2)+1) * cx
            A[4*i+1, (2*i):(2*i+2)] = [-r/L, r/L]
            A[4*i+1, (2*i):(2*i+2)] *= dt*8*ct
            A[(4*i+2):(4*i+4), (2*i):(2*i+2)] = np.eye(2, 2) * pcf
            b[4*i:(4*i+2)] = [cx*gdu[0,i], gdu[1,i]*ct]
            b[(4*i+2):(4*i+4)] = self.prev_cmd_vel[:, i] * pcf

        constrt = [v+M*s >= v_min, v+M*s <= v_max, s<=1, s>=0]
        obj = cp.Minimize(cp.sum_squares(A * v - b))
        prob = cp.Problem(obj, constrt)
        prob.solve()

        vel = v.value.astype(int).reshape([N,2]).T
        vel[:, np.linalg.norm(gdu, axis=0)<th] = 0

        return vel

    def go_to_goal(self, goal, th, cx=1, cy=1, ct=0.02):
        '''
        goal: 2-by-N or 3-by-N matrix
        th: threshold of goal region
        cx: weight of goal in x direction
        cy: weight of goal in y direction
        ct: weight of goal in heading angle 
        '''
        N = self.N
        dx = np.zeros([3, N])
        goal_th = np.zeros(N)
        vel = np.zeros([2, N])

        if not self.dts: return vel

        diff = goal[0:2, :] - self.pose[0:2, :]
        if goal.shape[0] < 3:
            goal = np.vstack((goal, np.arctan2(diff[1, :], diff[0, :])))

        dx[0:2, :] = goal[0:2, :] - self.pose[0:2, :]
        goal_th = np.arctan2(dx[1, :], dx[0, :])
        goal_th = self.utils.wrap_goal(goal_th, self.pose[2, :])
        if goal.shape[0] == 3:
            dx[2, :] = self.utils.wrap_pi(goal[2, :] - self.pose[2, :])
            dis_norm = np.linalg.norm(dx[0:2, :], axis=0)
            wt = np.zeros(N)
            dix_idx = dis_norm > th
            wt[dix_idx] = np.abs(dis_norm[dix_idx] - th) / dis_norm[dix_idx]
            goal_th = wt * goal_th
            goal_th += (1-wt) * goal[2, :]
        dx[2, :] *= ct
        dis = np.linalg.norm(dx, axis=0)

        if np.linalg.norm(dis) < th:
            return vel

        (r, L, a_s, pcf, lcf) = (0.002, 0.28, 1e-3, 1e-6, 1.0)
        (v_min, v_max) = (50, 120)
        M = v_max + v_min

        v = cp.Variable(2*N)
        s = cp.Variable(2*N, integer=True)
        A = np.zeros([5*N, 2*N])
        b = np.zeros(5*N)

        for i in range(N):
            theta = self.pose[2, i]
            dt = self.dts[i]*2
            if not dt: continue

            A[(5*i):(5*i+2), (2*i):(2*i+2)] = dt * r/2 * np.array([
                [cx*np.cos(theta), cx*np.cos(theta)],[cy*np.sin(theta), cy*np.sin(theta)]])
            A[5*i+2, (2*i):(2*i+2)] = [-r/L, r/L]
            A[5*i+2, (2*i):(2*i+2)] *= dt*a_s*10
            A[(5*i+3):(5*i+5), (2*i):(2*i+2)] = np.eye(2, 2) * pcf
            b[5*i:(5*i+3)] = [cx*dx[0,i], cy*dx[1,i], (goal_th[i]-theta)*a_s]
            b[(5*i+3):(5*i+5)] = self.prev_cmd_vel[:, i] * pcf

        constrt = [v+M*s >= v_min, v+M*s <= v_max, s<=1, s>=0]
        obj = cp.Minimize(cp.sum_squares(A * v - b))
        prob = cp.Problem(obj, constrt)
        prob.solve()
        #  rospy.logwarn("solution v is (%f, %f)" % tuple(v.value))

        vel = v.value.astype(int).reshape([N,2]).T

        vel[:, np.linalg.norm(dis, axis=0)<th] = 0

        return vel

    def project_du(self, du):
        x = self.pose
        t = 2
        goal = x.copy()
        goal[0, :] += du[0, :] * np.cos(x[2, :]) * t
        goal[1, :] += du[0, :] * np.sin(x[2, :]) * t
        goal[2, :] += du[1, :] * t
        return goal

    def align_and_connect(self, state):
        if state == 0:
            du = self.ctl.line_x(self.pose)
            #  print "distance: ", (self.pose[1, :] - np.mean(self.pose[1,:]))
            vel = self.fit_command_vel(du, 1e-3, cx=1, ct=1.0)
            if np.linalg.norm(vel) < 1e-3:
                state = 1
        elif state == 1:
            vel = np.zeros([2, self.N])
            if np.linalg.norm(self.vel) < 1e-2:
                state = 2
        elif state == 2:
            du = self.ctl.rend_x(self.pose)
            vel = self.fit_command_vel(du, 1e-3, cx=1, ct=1.0)
            if np.linalg.norm(vel) < 1e-2:
                state = 3
        elif state == 3:
            vel = np.zeros([2, self.N])
        return vel, state

    def connect_and_disconnect(self, state, lx, goal):
        # goal is 2-by-N
        vel = np.zeros([2, self.N])
        if state == 0:
            du = self.ctl.connect_along_x(self.pose)
            vel = self.fit_command_vel(du, 1e-3, cx=1, ct=1.0)
            if np.min(self.pose[0, :]) > lx:
                state = 1
                self.ctl.status[:] = 0
        elif state == 1:
            vel = np.zeros([2, self.N])
            if np.linalg.norm(self.vel) < 1e-3:
                state = 2
        elif state == 2:
            # already crossed the gap
            du = self.ctl.disconnect(self.pose)
            vel = self.fit_command_vel(du, 1e-3, cx=1, ct=1.0)
            if np.linalg.norm(vel) < 1e-3:
                state = 3
        elif state == 3:
            # disconnected
            vel = np.zeros([2, self.N])
            if np.linalg.norm(self.vel) < 1e-3:
                state = 4
        elif state == 4:
            # go to new goal position
            vel = self.go_to_goal(goal, 1e-3)
            if np.linalg.norm(vel) < 1e-3:
                state = 5

        return vel, state

    def keyboard_input_lr(self):
        du = np.zeros([2, self.N])
        txt = raw_input("Input cmd_vel in format [left right]: ")
        try:
            txt = txt.split()
            [l, r] = [int(tx) for tx in txt]
            du[0, :] = l
            du[1, :] = r
        except:
            return None

        return du

    def stop(self):
        return np.zeros([2, self.N])

