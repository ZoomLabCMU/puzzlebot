import numpy as np

class Utils:
    # Util functions
    def inv_g(self, g):
        #  assert(g.shape[0] == 3 or g.shape[0] == 4)
        #  assert(g.shape[1] == 3 or g.shape[1] == 4)

        if g.shape[0] != 3 or g.shape[1] != 3:
            print "We only handle 3x3 now"
            return
        
        g_inv = np.eye(3, 3)
        g_inv[0:2, 0:2] = g[0:2, 0:2].T
        g_inv[0:2, 2] = -g[0:2, 0:2].T.dot(g[0:2, 2])
        return g_inv

    def adjoint(self, g):
        if g.shape[0] != 3 or g.shape[1] != 3:
            print "We only handle 3x3 now"
            return
        
        ad = g.copy()
        ad[0, 2] = g[1, 2]
        ad[1, 2] = -g[0, 2]
        return ad

    def getR(self, th):
        return np.array([[np.cos(th), -np.sin(th)],
                        [np.sin(th), np.cos(th)]])

    def get_g(self, x):
        '''
        x is a 1d vector
        '''
        g = np.eye(3, 3)
        g[0:2, 0:2] = getR(x[2])
        g[0:2, 2] = x[0:2]
        return g

    def get_pair_dis(self, x, id_pairs, contact_pairs):
        M = len(contact_pairs)
        dis = np.zeros(M)

        for m in range(M):
            ids = id_pairs[:, m]
            cp = contact_pairs[m]
            x0 = self.getR(x[2, ids[0]]).dot(cp[:,0])
            x0 += x[0:2, ids[0]]
            x1 = self.getR(x[2, ids[1]]).dot(cp[:,1])
            x1 += x[0:2, ids[1]]
            dis[m] = np.linalg.norm(x0 - x1)

        return dis

    def is_in_pi(self, a):
        return (a<np.pi and a>-np.pi)

    def is_in_pi_2(self, a):
        return (a<np.pi/2 and a>-np.pi/2)
        
    def wrap_pi(self, a):
        return ((a + np.pi) % (2*np.pi) - np.pi)

    def wrap_pi_2(self, a):
        return ((a + np.pi/2) % np.pi - np.pi/2)

    def wrap_goal(self, goal, theta):
        '''
        goal: nd array, theta: nd array
        '''
        ng = goal.copy()

        tmp = self.wrap_pi(goal+np.pi)
        if not np.isscalar(ng):
            idx = np.abs(tmp - theta) < np.pi/2
            ng[idx] = tmp[idx]
        elif np.abs(tmp - theta) < np.pi/2:
            ng = tmp

        return ng

    def project_robot_dis(self, t, cp):
        '''
        t: len=2 vector
        cp: 2-by-2 matrix, colm i is contact pair for ri
        '''
        x = np.zeros([2, 2])
        for i in range(2):
            x[:, i] = - self.getR(t[i]).dot(cp[:, i])
        return np.linalg.norm(x[:, 0] - x[:, 1])
