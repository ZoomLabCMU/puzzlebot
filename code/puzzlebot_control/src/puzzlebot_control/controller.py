import numpy as np
import quadprog 
from planner import Planner
from utils import Utils

class Controller:
    def __init__(self, N, vlim=0.02, wlim=0.5, vcf=1.0, wcf=1.0, knob_length=7.5, body_length=34, padding_length=1, barrier_gain=1e-1, safe_radius_ratio=1.5, obstacle_distance=0.1):
        self.N = N
        self.vlim = vlim
        self.wlim = wlim
        self.vcf = vcf
        self.wcf = wcf

        # input params are in mm
        self.knob_length = knob_length
        self.body_length = body_length
        self.padding_length = padding_length

        self.Lx = (self.knob_length*1.5 + self.padding_length) / 1e3
        self.L = self.body_length / 1e3
        self.r = self.knob_length / 2 / 1e3

        self.Q = None
        self.ly = None
        self.status = np.zeros(N)

        self.utils = Utils()

        self.prev_du = np.zeros([2, N])

    def cap_dx(self, _dx):
        dx = np.array(_dx, copy=True)

        idx = np.abs(dx) > self.vlim
        dx[idx] = self.vlim * np.sign(dx[idx])

        return dx

    def cap(self, _du):
        du = np.array(_du, copy=True)
        du[0, :] *= self.vcf
        du[1, :] *= self.wcf

        idx = np.abs(du[0, :]) > self.vlim
        du[0, idx] = self.vlim * np.sign(du[0, idx])
        idx = np.abs(du[1, :]) > self.wlim
        du[1, idx] = self.wlim * np.sign(du[1, idx])

        return du

    def go_to_goal(self, x, goal, weights=[1,1], eth=1e-3):
        '''
        x: 3-by-N matrix, goal: 2-by-N matrix or 3-by-N
        '''
        N = x.shape[1]
        weights = np.array([weights]).T
        dis_th = 0.1
        du = np.zeros([2, N])

        diff = goal[0:2, :] - x[0:2, :]
        if np.linalg.norm(diff * weights) < eth:
            return du
        if goal.shape[0] < 3:
            goal = np.vstack((goal, np.arctan2(diff[1, :], diff[0, :])))
        diff = goal - x

        for i in range(N):
            th = x[2, i]
            dif_i = goal[:, i] - x[:, i]
            dif_i[2] = self.utils.wrap_pi(goal[2, i])
            dif_i[2] -= th
            J_inv = np.array([[np.cos(th), np.sin(th), 0], [0, 0, 1]])
            du[:, i] = J_inv.dot(dif_i)

        du[:, np.linalg.norm(diff[0:2, :] * weights) < eth] = 0
        du = self.cap(du)
        if N == 1:
            du = du.flatten()
        return du

    def line_x(self, x):
        '''
        x: 3-by-N matrix
        '''
        N = self.N
        goal = np.zeros([2, N])
        du = np.zeros([2, N])
        ly = self.ly
        if ly is None:
            ly = np.mean(x[1, :])
            self.ly = ly

        if np.linalg.norm(x[1, :] - ly) < 1e-3:
            return du
        
        goal[1, :] = ly

        gs = np.arange(N) * self.L * 2.5
        gs += np.mean(x[0, :]) - np.mean(gs)
        sidx = np.argsort(x[0, :])
        goal[0, sidx] = gs
        print 'goal:', goal
        du = self.go_to_goal(x, goal, weights=[0.1, 1])

        # add velocity limit
        du = self.cap(du)
        return du

    def rend_x(self, x):
        N = self.N
        goal = np.zeros([3, N])
        du = np.zeros([2, N])
        sidx = np.argsort(x[0, :])
        dif = np.diff(x[0, sidx])

        if np.all(dif < self.L+1e-3):
            return du

        goal[0, :] = np.mean(x[0, :])
        goal[1, :] = np.mean(x[1, :])
        du = self.go_to_goal(x, goal)

        du = self.cap(du)
        return du

    def forward_y(self, x, goal=[], vel=None):
        '''
        Input: x: 3-by-N matrix 
        Output: du: 1-by-2 vector
        '''
        N = self.N
        du = np.zeros([2, N])
        du[0, :] = self.vlim

        return du

    def align_heading(self, x, eth=1e-2):
        du = np.zeros([2, self.N])
        idx = np.abs(x[2, :]) > eth
        du[1, idx] = - x[2, idx]
        du = self.cap(du)
        return du

    def connect_along_x(self, x, th=2e-3):
        N = self.N
        if np.linalg.norm(x) < 1e-2:
            return None
        du = np.zeros([2, N])
        du[0, :] -= self.vlim
        
        sidx = np.argsort(x[0, :])

        for i in range(N-1):
            dis = np.abs(x[0, sidx[i]] - x[0, sidx[i+1]])
            if dis < (self.L + th):
                print("id: %d connected" % sidx[i])
                self.status[sidx[i]] = 1
                self.status[sidx[i+1]] = 1

        nidx = self.status > 0
        du[0, nidx] = self.vlim
        du[1, nidx] = - x[2, nidx]

        du = self.cap(du)
        return du

    def disconnect(self, x, th=1.5e-2):
        du = np.zeros([2, self.N])
        status = np.zeros(self.N)
        sidx = np.argsort(x[0, :])
        #  du[0, sidx] = np.linspace(-dis_vel, dis_vel, num=self.N)
        du[0, sidx] = -(np.mean(x[0,:]) - x[0,sidx])

        for i in range(self.N-1):
            dis = np.abs(x[0, sidx[i]] - x[0, sidx[i+1]])
            if dis > (self.L + th):
                print("id: %d disconnected" % sidx[i])
                status[sidx[i]] = 1
                status[sidx[i+1]] = 1

        #  du[0, status == 0] = np.linspace(-dis_vel, dis_vel, num=np.count_nonzero(status == 0))
        du[0, status > 0] = 0

        du = self.cap(du)
        return du

    def no_control(self, x, goal=[], dx=[]):
        return np.zeros([2, self.N])

