import pymap3d as pm
import numpy as np
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
    compute_preview = False; # compute _-step preview
    prev_length = 1

    _len_path = 0
    _path = 0
    _size_path = 0

    def __init__(self, pathfile="data/route/ParkingLotStraight.csv"):
        self._path = np.genfromtxt(self.pathfile, skip_header=1)
        self._size_path, _ = self._path.shape
        self._len_path = self._path[-1,2]

    # Assuming data is in dict defined in getData.gpsData.dataClean
    def step(self, data):
        # Convert GPS position to cg XY coordinate
        r_cg = cg_pos(data)
        # TODO CONTINUE

    def get_road_state(self, pos):
        pt_idx_min = min_idx(self.path[:,0:1] - matlib.repmat[pos, self._size_path, 1])
        # TODO CONTINUE
