import rospy
import numpy as np
import threading
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

rad2deg = lambda x: x*180/np.pi
normDeg = lambda x: np.pi - np.mod(np.pi-x,2*np.pi)

class Data:
    r_gps_cg = (.99, 0)
    x_gps_cg = .99
    y_gps_cg = 0
    Vx = 0
    Vy = 0
    Yaw = 0
    latitude = 0
    longitude = 0
    el = 0

    def __init__(self):
        rospy.Subscriber("/gps/fix", NavSatFix, self.updateGPS)
        rospy.Subscriber("/gps/odom", Odometry, self.updateOdom)
        self.lock = threading.Lock()

    def updateGPS(self,data):
        self.lock.acquire()
        self.latitude = (data.latitude)
        self.longitude = (data.longitude)
        self.el = data.altitude
        self.lock.release()

    def updateOdom(self,data):
        self.lock.acquire()
        self.YawRate = data.twist.twist.angular.z
        self.Vx = data.twist.twist.linear.x + self.y_gps_cg*self.YawRate
        self.Vy = data.twist.twist.linear.y + self.x_gps_cg*self.YawRate
        self.Yaw = np.arctan2(2*(data.pose.pose.orientation.w*data.pose.pose.orientation.z +
                                 data.pose.pose.orientation.x*data.pose.pose.orientation.y),
                            1-2*(data.pose.pose.orientation.y**2 +
                                 data.pose.pose.orientation.z**2))
        self.Yaw = normDeg(self.Yaw + np.pi/2)
        self.lock.release()

    def __str__(self):
        self.lock.acquire()
        out = "Lat: {}, Long: {}, El: {} \n Vx: {}, Vy: {}, Yaw: {}".format(
            self.latitude, self.longitude, self.el, self.Vx, self.Vy, self.Yaw)
        self.lock.release()
        return out

def run():
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
