from mcity_msg.msg import Control
from scipy.io import loadmat
from itertools import permutations
import numpy as np

class CC:
    Vold = 0
    eI = 0
    max = .45
    min = -.32
    def __init__(self, set, P = 0, I = 0, D = 0, dt = 0.02):
        self.P = P
        self.I = I
        self.D = D
        self.dt = dt
        self.set = set

    def PID(self, V):
        eP = self.set - V
        eD = (V - self.Vold)/self.dt
        self.Vold = V
        u = self.P*eP + self.D*eD + self.I*self.eI
        if (u > self.min and u < self.max): # If saturated dont integrate
            self.eI += eP*self.dt
        return u

    def controlSig(self,V):
        u = self.PID(V)
        F = Control()
        if u > 0:
            F.throttle_cmd = min(u, .42)
            F.brake_cmd = 0
        else:
            F.throttle_cmd = 0
            F.brake_cmd = min(-u, .35)
        return F

def recurPerm(accum, build, x):
    i = len(build)
    if i == len(x):
        accum.append(build)
        return accum
    for s in [1, -1]:
        temp = build[:]
        temp.append(s*x[i])
        accum = recurPerm(accum, temp, x)
    return accum

class LK:
    _u_max = 2.5*np.pi/14.8
    _u_min = -_u_max
    _M1 = .2
    _M2 = .5
    _M3 = .8
    _M4 = .9
    _b = .02
    _dt = .02
    _S1 = _M2 - _M1
    _S2 = _M4 - _M3

    def __init__(self, speed):
        self.info = loadmat("/home/laptopuser/mkz/data/pcis/lk_cinv_mph_"+str(speed)+".mat")
        self.info['n'], self.info['m'] = self.info['dyn_B'].shape
        self.info['V_d'] = np.transpose(np.array(recurPerm([],[], [x[0] for x in self.info['bnd_Ed']])))
        _, k = self.info['V_d'].shape
        l, _ = self.info['W_A'].shape
        self.info['A'] = np.repeat(self.info['W_A']@self.info['dyn_B'],k).reshape(l, k)
        self.MOld = 0

    def barrierMag(self, x):
        assert len(x) == self.info['n'], 'x dim is wrong'
        return np.max((self.info['W_A']@x).reshape(len(self.info['W_A']),1)/self.info['W_b'])

    def userPredictM(self, x, u, k):
        xN = self.info['dyn_A']@x + self.info['dyn_B']*u + self.info['dyn_Ep']*k
        dE = self.info['dyn_Ed']@self.info['V_d']
        l, k = dE.shape
        xNe = np.repeat(xN,k).reshape(l, k)
        xNe += dE
        M = 0
        for i in range (k):
            Mt = self.barrierMag(xNe[:,i])
            if Mt > M:
                M = Mt
        return M

    def magMerge(self, M, uOpt,uD):
        dM = (M-self.MOld)/self._dt;
        self.MOld = M;
        if (M < self._M1):
            c = 0
        elif (M < self._M2):
            c = max(min(self._b*dM*(M-self._M1)/self._S1, 1), 0)
        elif (M < self._M3):
            c = max(min(self._b*dM, 1), 0)
        elif (M < self._M4):
            c = max(min(self._b*dM+(M-self._M3)/self._S2, 1), 0)
        else:
            c = 1;
        u = c*uOpt + (1 - c)*uD
        return u, c

    def optIn(self, x, p): #NEEED TO CONFIRM WITH MATLAB
        assert len(x) == self.info['n'], 'x dim is wrong'
        ins = self.info['dyn_A']@x + self.info['dyn_Ep']*p
        cf = self.info['W_A']@ins
        accum = self.info['W_b'] - cf
        _, k = self.info['V_d'].shape
        l, _ = self.info['W_A'].shape
        accum = np.repeat(accum,k).reshape(l, k)
        distEffect = self.info['W_A']@self.info['dyn_Ed']@self.info['V_d']
        b = accum - distEffect
        u1 = self._u_min
        u2 = self._u_max
        dist = u2 - u1
        f1 = 0
        f2 = 0
        um1 = 0
        um2 = 0
        while (dist >= 1e-3):
            um1 = u1 + 1/3*dist
            um2 = u1 + 2/3*dist
            accum1 = cf+self.info['W_A']@self.info['dyn_B']*um1
            accum1 = np.repeat(accum1,k).reshape(l, k)
            f1 = np.max((accum1+distEffect)/self.info['W_b'])
            accum2 = cf+self.info['W_A']@self.info['dyn_B']*um2
            accum2 = np.repeat(accum2,k).reshape(l, k)
            f2 = np.max((accum2+distEffect)/self.info['W_b'])
            if f1 < f2:
                u2 = um2
            else:
                u1 = um1
            dist = u2-u1
        return (u1+u2)/2

    def supervise(self, x, u, p):
        uOpt = self.optIn(x,p)
        M = self.userPredictM(x, u, p)
        uOut, c = self.magMerge(M, uOpt, u)
        return uOut, uOpt, c
