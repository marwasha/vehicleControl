import threading
import rospy
from mcity_msg.msg import Control

class controlPublisher:
    '''A dumb loop that runs at 50hz to publish car control inline wiht the car'''
    def __init__ (self, hz = 50):
        '''Intalizes the needed ros and threading modules'''
        self.pub = rospy.Publisher('/mkz_bywire_intf/control', Control, queue_size=10)
        self.lock = threading.Lock()
        self.Control = Control()
        self.count = 0
        self.Control.timestamp = 0
        self.Control.count = self.count
        self.Control.brake_cmd = 0
        self.Control.throttle_cmd = 0
        self.Control.steering_cmd = 0
        self.Control.gear_cmd = 0
        self.Control.turn_signal_cmd = 0
        self.hz = hz
        self.t = threading.Thread(target=self.push, daemon = True)

    def start(self):
        '''starts the worker thread'''
        self.rate = rospy.Rate(self.hz)
        self.t.start()

    def update(self,control: Control):
        '''A way to update the control being published'''
        self.lock.acquire()
        self.Control = control
        self.lock.release()

    def push(self):
        '''The worker thread which contantly publishes this classes control'''
        while not rospy.is_shutdown():
            self.lock.acquire()
            self.pub.publish(self.Control)
            self.lock.release()
            self.rate.sleep()
