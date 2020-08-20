import pymap3d as pm
import numpy as np
import scipy as sp
import scipy.optimize
import scipy.interpolate
import json
from numpy import matlib

lat0 = 42.30095833
long0 = -83.69758056
h0 = 0
wgs84 = pm.utils.Ellipsoid(model='wgs84')

# Assuming data is in dict defined in getData.gpsData.dataClean
def cg_pos(data):
    x, y, _ = pm.geodetic2enu(data['latitude'], data['longitude'], data['el'],
                              lat0, long0, h0, ell = wgs84)
    xC, yC = (data["x_gps_cg"]*np.cos(data["Yaw"]) -
                data["y_gps_cg"]*np.sin(data["Yaw"]) ,
              data["x_gps_cg"]*np.sin(data["Yaw"]) +
                data["y_gps_cg"]*np.cos(data["Yaw"]))
    return x + xC, y + yC

# return index of row in v with smallest 2-norm
def min_idx(v):
    return np.argmin(np.sum(np.square(v), axis=1))

class road:
    circular = 0;
    N_path = 3;             # number of points in each direction to fit
    N_interp = 10;          # number of subdivisions to find closes point
    dt = 0.1;               # time step for computing derivatives
    compute_preview = True; # compute _-step preview
    prev_length = 1

    _len_path = 0
    _path = 0
    _size_path = 0

    def __init__(self, pathfile="data/route/ParkingLotStraight.csv"):
        self._path = np.genfromtxt(pathfile, delimiter=",", skip_header=1)
        self._size_path, _ = self._path.shape
        self._len_path = self._path[-1,2]

    # Assuming data is in dict defined in getData.gpsData.dataClean
    def step(self, data):
        # Convert GPS position to cg XY coordinate
        r_cg = cg_pos(data)
        rc, drc, kappa, road_left, s = self.get_road_state(r_cg)
        road_state = {}
        road_state['car_x'] = r_cg[0]
        road_state['car_y'] = r_cg[1]

        road_state['road_x'] = rc[0]
        road_state['road_y'] = rc[1]

        road_state['d_road_x'] = drc[0]
        road_state['d_road_y'] = drc[1]

        road_state['kappa'] = kappa;

        ###Transformation of ACC/LK State
        #Unit Vector of Road
        road_unit = np.array([road_state['d_road_x'], road_state['d_road_y']])
        road_unit = road_unit/np.linalg.norm(road_unit)
        #Relaltive Position
        rel_pos = np.array([road_state['car_x'] - road_state['road_x'],
                            road_state['car_y'] - road_state['road_y']])
        #Vel in global frame
        global_vel = np.array([[data['Vx']*np.cos(data['Yaw']) - data['Vy']*np.sin(data['Yaw'])],
                               [data['Vx']*np.sin(data['Yaw']) + data['Vy']*np.cos(data['Yaw'])]])
        #Yaw Angles
        normDeg = lambda x: np.pi - np.mod(np.pi-x,2*np.pi)

        road_yaw = np.arctan2(road_unit[1], road_unit[0])
        global_yaw = normDeg(normDeg(data['Yaw']) + np.arctan2(data['Vy'], data['Vx']))

        dPsi = normDeg(global_yaw - road_yaw)[0]

        #Assign
        lk_acc_state = {}
        lk_acc_state['y'] = np.linalg.det(np.block([road_unit, rel_pos]))    # FIX
        lk_acc_state['dy'] = np.linalg.det(np.block([road_unit, global_vel]))# FIX
        lk_acc_state['mu'] = data['Vx']
        lk_acc_state['nu'] = data['Vy']
        lk_acc_state['dPsi'] = dPsi
        lk_acc_state['r'] = data['YawRate']
        lk_acc_state['h'] = 8
        lk_acc_state['r_d'] = road_state['kappa'] * np.linalg.norm(global_vel)
        lk_acc_state['kappa'] = kappa
        # todo add preview and test
        # Old preview of points, we want kappa
        '''
        if self.compute_preview:
            prev_pts = np.zeros((2, self.prev_length))
            for ds in range(self.prev_length):
                s_prev = np.mod(s+ds+1, self._len_path)
                prev_pts[:,ds] = self.get_pos(s_prev) - r_cg
            yaw = global_yaw
            prev_pts = np.array[[np.cos(yaw), np.sin(yaw)],
                                [-np.sin(yaw), np.cos(yaw)]]@prev_pts
            prev = prev_pts[2,:]
        else:
            prev = np.zeros((1, self.prev_length))
        '''
        if self.compute_preview:
            prev = np.zeros((1, self.prev_length))
            for ds in range(self.prev_length):
                s_prev = np.mod(s+ds+1, self._len_path)
                _, _, prev[0,ds] = self.get_pos(s_prev)
        else:
            prev = np.zeros((1, self.prev_length))
        return lk_acc_state, road_left[0], prev

    def get_road_state(self, veh_pos):
        # Find Closest Road Point
        pt_idx_min = min_idx(self._path[:,0:2] - matlib.repmat(veh_pos, self._size_path, 1))
        if self.circular:
            # Path is a loop
            # Find points near it within N
            ival_mid = self.N_path
            interp_ival = np.mod(np.r_[pt_idx_min-self.N_path:pt_idx_min+self.N_path + 1],
                                       self._size_path-1);
            s_interp = self._path[interp_ival,2]
            s_interp = s_interp - self._len_path*(s_interp > s_interp[-1])
        else:
            # Path is not loop
            ival_sta = max(0, pt_idx_min-self.N_path)
            ival_end = min(self._size_path-1, pt_idx_min+self.N_path)
            ival_mid = pt_idx_min-ival_sta;
            interp_ival = np.r_[ival_sta:ival_end+1] # points to interpolate
            s_interp = self._path[interp_ival, 2]
        x_spline = sp.interpolate.CubicSpline(s_interp,self._path[interp_ival,0])
        y_spline = sp.interpolate.CubicSpline(s_interp,self._path[interp_ival,1])

        interp_pts = np.linspace(s_interp[max(0,ival_mid-1)],
                                 s_interp[min(len(s_interp)-1, ival_mid+1)],
                                 self.N_interp)
        traj_pts = np.transpose(np.array([x_spline(interp_pts), y_spline(interp_pts)]))
        s_idx_min = min_idx(traj_pts - matlib.repmat(veh_pos, self.N_interp, 1))

        sFun = lambda x: np.linalg.norm([veh_pos[0]-x_spline(x),
                                         veh_pos[1]-y_spline(x)])
        s = scipy.optimize.fmin(func=sFun, x0 = interp_pts[s_idx_min], disp=False)

        road_left = self._len_path - s
        # Pos
        rc = np.array([x_spline(s), y_spline(s)])
        # 1st derivative of position
        drc = (np.array([x_spline(s+self.dt), y_spline(s+self.dt)]) - rc)/self.dt;
        # 2nd derivative of position
        ddrc = (np.array([x_spline(s+self.dt) + x_spline(s-self.dt),
                          y_spline(s+self.dt) + y_spline(s-self.dt)])
                 - 2*rc)/self.dt**2
        # curvature
        kappa = np.linalg.det(np.block([drc,ddrc]))/np.linalg.norm(drc)**3
        return rc, drc, kappa, road_left, s

    def get_pos(self,s):
        idx_closest = min_idx(np.reshape(self._path[:,2], (len(self._path[:,2]),1)) - s)

        if self._path[idx_closest,2] <= s :
            idx1 = idx_closest
        else:
            idx1 = idx_closest-1

        s0 = self._path[idx1,2]
        x0 = self._path[idx1,0:2]

        s1 = self._path[idx1+1,2]
        x1 = self._path[idx1+1,0:2]

        x = x0 * (s1-s)/(s1-s0) + x1 * (s-s0)/(s1-s0);
        rc, drc, kappa, _, _ = self.get_road_state(x);

        return rc, drc, kappa

if __name__ == '__main__':
    import csv
    # Rospy free test if step works for one step
    test = road()
    file = "data/raw/ParkingLotSwerve.csv"
    with open(file, 'r') as dataSource:
        reader = csv.DictReader(dataSource, quoting=csv.QUOTE_NONNUMERIC)
        dataTest = []
        for row in reader:
            dataTest.append(row)
    lk_acc_state, road_left, prev = test.step(dataTest[0])
    csv_columns = list(lk_acc_state.keys())
    results = "data/testSwerve.csv";
    with open(results, 'w') as dataOut:
        writer = csv.DictWriter(dataOut,
                                fieldnames=csv_columns,
                                quoting=csv.QUOTE_NONNUMERIC)
        writer.writeheader()
        for info in dataTest:
            lk_acc_state, road_left, prev = test.step(info)
            writer.writerow(lk_acc_state)
