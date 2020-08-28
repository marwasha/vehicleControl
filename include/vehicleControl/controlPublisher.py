import threading
import rospy
from mcity_msg.msg import Control

class controlPublisher:
    '''A dumb loop that runs at 50hz to publish car control inline wiht the car'''
    def __init__ (self, hz = 50):
        '''Intalizes the needed ros and threading modules'''
        self.rate = rospy.Rate(hz)
        self.pub = rospy.Publisher("/mkz_bywire_intf/control", Control, queue_size = 10)
        self.lock = threading.Lock()
        self.control = Control()
        self.t = threading.Thread(target=self.push, daemon = True)

    def start(self):
        '''starts the worker thread'''
        self.t.start()

    def update(self,control: Control):
        '''A way to update the control being published'''
        self.lock.acquire()
        self.control = control
        self.lock.release()

    def push(self):
        '''The worker thread which contantly publishes this classes control'''
        while not rospy.is_shutdown():
            self.lock.acquire()
            self.pub.publish(self.control)
            self.lock.release()
            self.rate.sleep()
