### CANT READ KEYBOARD INPUTS, ABBANONDING TILL LATER


# A way to control the vehicle with the keyboard for simple tests
# if I could do this overagain, I would make an input base class then build steeringWheel
# and keyboard off of it. Since I am trying to get stuff to work, I plan to keep these
# independant to ensure nothing with steeringWheel gets broken

import pygame
import threading
import rospy
from mcity_msg.msg import Control

pygame.init()

class keyboard:
    supervise = False
    LK = False
    CC = False
    quit = False
    force = 0

    def __init__(self, rate=20):
        self.rate = rate
        self.T = 1/self.rate
        self.lock = threading.Lock()
        self.control = Control()
        self.control.count = 0
        self.t = threading.Thread(target=self.main, daemon = True)
        self.t.start()

    def main(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()
            if (self.quit):
                return

    def update(self): # Only gets true value once is moved
        self.lock.acquire()
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    self.control.gear_cmd = 1
                elif event.key == pygame.K_w:
                    self.control.gear_cmd = 2
                elif event.key == pygame.K_e:
                    self.control.gear_cmd = 3
                elif event.key == pygame.K_r:
                    self.control.gear_cmd = 4
                elif event.key == pygame.K_u:
                    self.supervise = not self.supervise
                elif event.key == pygame.K_BACKSPACE:
                    self.quit = True
                elif event.key == pygame.K_o:
                    self.CC = not self.CC
                    if (self.CC):
                        print("CC on")
                    else:
                        print("CC off")
                elif event.key == pygame.K_p:
                    self.LK = not self.LK
                    if (self.LK):
                        print("LK on")
                    else:
                        print("LK off")
                elif event.key == pygame.K_UP:
                    self.force += .01
                elif event.key == pygame.K_DOWN:
                    self.force -= .01
                elif event.key == pygame.K_LEFT:
                    self.control.steering_cmd += .01
                elif event.key == pygame.K_RIGHT:
                    self.control.steering_cmd -= .01
        self.control.timestamp = rospy.get_time()*0
        self.control.count += 1
        self.control.throttle_cmd = max(0, self.force)
        self.control.brake_cmd = max(0, -self.force)
        if self.control.brake_cmd < .001:
            self.control.brake_cmd = 0
        self.lock.release()

    def getControl(self):
        self.lock.acquire()
        out = self.control
        self.lock.release()
        return out

    def getFlags(self):
        self.lock.acquire()
        CC = self.CC
        LK = self.LK
        quit = self.quit
        self.lock.release()
        return CC, LK, quit

    def __del__(self):
        pygame.quit()

    def __str__(self):
        self.lock.acquire()
        out = "Angle:{} Gas:{} brake:{} \n".format(
                self.control.steering_cmd, self.control.throttle_cmd, self.control.brake_cmd)
        self.lock.release()
        return out


if __name__ == '__main__':
    rospy.init_node('Test', anonymous=True)
    testKey = keyboard()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        pygame.event.pump()
        print(pygame.key.get_pressed())
        rate.sleep()
