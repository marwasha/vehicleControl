from mcity_msg.msg import Control

class accPID:
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
        if (u > self.min and u < self.max):
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
