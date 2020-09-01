import rospy
import numpy as np
import threading
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

rad2deg = lambda x: x*180/np.pi
normDeg = lambda x: np.pi - np.mod(np.pi-x,2*np.pi)

class gpsData:
    '''A class which handles the gps data inputs

    Has callbacks to handle the different rostopics subsribers  which then update
    the classes on data. Then uses threading locks to make sure one can access the
    data in a safe way
    '''
    r_gps_cg = (.99, 0)
    dataClean = {'Vx': 0,
                 'Vy': 0,
                 'Yaw': 0,
                 'YawRate': 0,
                 'latitude':0,
                 'longitude': 0,
                 'el': 0,
                 'x_gps_cg': .99,
                 'y_gps_cg': 0}

    def __init__(self):
        '''Starts the subsriber threads'''
        rospy.Subscriber("/gps/fix", NavSatFix, self.updateGPS)
        rospy.Subscriber("/gps/odom", Odometry, self.updateOdom)
        self.lock = threading.Lock()

    def updateGPS(self,data):
        '''Callback to update class GPS data'''
        self.lock.acquire()
        self.dataClean['latitude'] = (data.latitude)
        self.dataClean['longitude'] = (data.longitude)
        self.dataClean['el'] = data.altitude
        self.lock.release()

    def updateOdom(self,data):
        '''Callback to update class Odometry data'''
        self.lock.acquire()
        YawRate = data.twist.twist.angular.z
        self.dataClean['YawRate'] = YawRate
        self.dataClean['Vx'] = data.twist.twist.linear.x + \
                               self.dataClean['y_gps_cg']*YawRate
        self.dataClean['Vy'] = data.twist.twist.linear.y + \
                               self.dataClean['x_gps_cg']*YawRate
        Yaw = np.arctan2(2*(data.pose.pose.orientation.w*data.pose.pose.orientation.z +
                            data.pose.pose.orientation.x*data.pose.pose.orientation.y),
                       1-2*(data.pose.pose.orientation.y**2 +
                            data.pose.pose.orientation.z**2))
        self.dataClean['Yaw'] = normDeg(Yaw) # Used to have plus pi/2, why?
        self.lock.release()

    def getDataClean(self):
        '''Locks the information to acquire the data'''
        self.lock.acquire()
        out = self.dataClean
        self.lock.release()
        return out

    def __str__(self):
        ''' Allows for one to proint gps data to the terminal'''
        self.lock.acquire()
        out = "Lat: {}, Long: {}, El: {} \n Vx: {}, Vy: {}, Yaw: {}".format(
            self.dataClean['latitude'], self.dataClean['longitude'],
            self.dataClean['el'], self.dataClean['Vx'],
            self.dataClean['Vy'], self.dataClean['Yaw'])
        self.lock.release()
        return out

def run():
    '''A small function for testing'''
    rospy.init_node('Laptop', anonymous=True)
    data = dataIn()
    pub = rospy.Publisher('/mkz_bywire_intf/control', Control, queue_size=10)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        rate.sleep


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
