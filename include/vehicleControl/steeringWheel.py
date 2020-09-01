# TODO: get the FF to work properly
# TODO: Figure out how to have it that we only update buttons when processed
# TODO: Figure out control passing scheme

import pygame
import numpy as np
import evdev
import math
import threading
import rospy
from mcity_msg.msg import Control
pygame.init()
pygame.joystick.init()

"""
Logitech Buttons and Axis
A0 - Wheel
A1 - Gas
A2 - Brake
A3 - Clutch
B0  - A
B1  - B
B2  - X
B3  - Y
B6  - Bars
B7  - Boxs
B8  - RSB
B9  - LSB
B10 - Xbox
"""

class Wheel:
    k = 10 # For virtual wall
    errorOld = 0 # For virtual wall
    effect_id0 = -1
    effect_id1 = -1
    setup = False
    go = False
    flip = 0
    supervise = False
    LK = False
    CC = False
    quit = False

    def __init__(self, rate = 20,name = -1, channel = 0):
        self.rate = rate
        self.T = 1/self.rate
        self.lock = threading.Lock()
        self.control = Control()
        self.control.count = 0
        assert pygame.joystick.get_count() != 0, 'No joystick Connected'
        self.js = pygame.joystick.Joystick(channel)
        self.js.init()
        # If no dev given first EV_FF capable event device (should be wheel)
        if name == -1:
            found = False
            for name in evdev.list_devices():
                self.dev = evdev.InputDevice(name)
                if evdev.ecodes.EV_FF in self.dev.capabilities():
                    found = True
                    break
            if not found:
                error('Could not connect FF to wheel')
        else:
            self.dev = evdev.InputDevice(name)
        self.update()
        #Starts a thread to keep updating the wheel
        self.t = threading.Thread(target=self.main, daemon = True)
        self.t.start()

    #Function to keep updating the wheel at set rate
    def main(self):
        rate = rospy.Rate(self.rate)
        print("Please nudge the wheel, throttle, and brake to init values")
        while not self.setup:
            self.update()
            self.lock.acquire()
            if (self.control.steering_cmd != 0 and
                self.control.throttle_cmd != .45/2 and
                self.control.brake_cmd != .32/2 and
                self.go):
                self.setup = True
            self.lock.release()
        print("Wheel is now setup")
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()
            if (self.quit):
                return

    def update(self): # Only gets true value once is moved
        self.lock.acquire()
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == 0:           # A
                    self.control.gear_cmd = 1
                elif event.button == 1:         # B
                    self.control.gear_cmd = 2
                elif event.button == 2:         # X
                    self.control.gear_cmd = 3
                elif event.button == 3:         # Y
                    self.control.gear_cmd = 4
                elif event.button == 6:         # Bars
                    self.supervise = not self.supervise
                elif event.button == 7:         # Boxs
                    self.quit = True
                elif event.button == 8:         # RSB
                    self.CC = not self.CC
                    if (self.CC):
                        print("CC on")
                    else:
                        print("CC off")
                elif event.button == 9:         # LSB
                    self.LK = not self.LK
                    if (self.LK):
                        print("LK on")
                    else:
                        print("LK off")
                elif event.button == 10:        # XBox
                    self.go = True
        self.control.timestamp = rospy.get_time()*0
        self.control.count += 1
        self.control.steering_cmd = -self.js.get_axis(0)*2.5*np.pi
        self.control.throttle_cmd = -(self.js.get_axis(1)-1)/2*.45
        self.control.brake_cmd = -(self.js.get_axis(2)-1)/2 *.32
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

    def makeEffect(self, level):
        # Level bounded by -1 and 1
        force = int(math.floor(level * 32767))
        envop = evdev.ff.Envelope(0, 0, 0, 0)
        const = evdev.ff.Constant(force, envop)
        effectType = evdev.ff.EffectType(ff_constant_effect=const)
        self.effect =  evdev.ff.Effect(evdev.ecodes.FF_CONSTANT, -1,
                                  0x6000, evdev.ff.Trigger(0,0),
                                  evdev.ff.Replay(int((1000/self.rate)*1.2), 0), effectType)
    def runEffect(self):
        if self.flip == 0:
            if self.effect_id0 != -1:
                self.dev.erase_effect(self.effect_id0)
            self.effect_id0 = self.dev.upload_effect(self.effect)
            self.dev.write(evdev.ecodes.EV_FF, self.effect_id0, 1)
            self.flip = 1
        elif self.flip == 1:
            if self.effect_id1 != -1:
                self.dev.erase_effect(self.effect_id1)
            self.effect_id1 = self.dev.upload_effect(self.effect)
            self.dev.write(evdev.ecodes.EV_FF, self.effect_id1, 1)
            self.flip = 0

    def ff(self, level):
        level = min(max(level,-1),1)
        self.makeEffect(level)
        self.runEffect()

    def virtualWall(self, set, pos):
        error = pos-set
        errorDot = (error-self.errorOld)/self.T
        F = -self.k*(error + self.T/2*errorDot) # This removes wall chatter
        self.ff(F)
        self.errorOld = error

    def __del__(self):
        pygame.joystick.quit()
        pygame.quit()

    def __str__(self):
        self.lock.acquire()
        out = "Angle:{} Gas:{} brake:{} \n".format(
                self.angle, self.throttle, self.brake)
        out += "A:{} B:{} X:{} Y:{}".format(
                self.A, self.B, self.X, self.Y)
        self.lock.release()
        return out

if __name__ == '__main__':
    rospy.init_node('Test', anonymous=True)
    testWheel = Wheel()
    testWheel.level = 1
    while not rospy.is_shutdown():
        rospy.spin()
