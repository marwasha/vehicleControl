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

class Wheel:
    rate = 50
    effect_id = -1
    setup = False
    def __init__(self, name = -1, channel = 0):
        self.lock = threading.Lock()
        self.control = {}
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
            if (self.angle != 0 and self.throttle != .5 and self.brake != .5):
                self.setup = True
            self.lock.release()
        print("Wheel is now setup")
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()

    def update(self): # Only gets true value once is moved
        pygame.event.get()
        self.lock.acquire()
        self.angle = self.js.get_axis(0)            # -1 to 1
        self.throttle = -(self.js.get_axis(1)-1)/2  #  0 to 1
        self.brake = -(self.js.get_axis(2)-1)/2     #  0 to 1
        self.A = self.js.get_button(0)              #  0 or 1
        self.B = self.js.get_button(1)              #  0 or 1
        self.X = self.js.get_button(2)              #  0 or 1
        self.Y = self.js.get_button(3)              #  0 or 1
        self.RSB = self.js.get_button(8)            #  0 or 1
        self.LSB = self.js.get_button(9)            #  0 or 1
        self.Bars = self.js.get_button(6)           #  0 or 1
        self.Boxs = self.js.get_button(7)           #  0 or 1
        self.xBox = self.js.get_button(10)          #  0 or 1
        self.control.timestamp = rospy.get_time()
        self.control.count += 1
        self.control.steering_cmd = self.angle*2.5*np.pi
        self.control.throttle_cmd = self.throttle*.45
        self.control.brake_cmd = self.brake*.32
        self.control.gear_cmd += self.A - self.B
        self.control.turn_signal_cmd = self.RSB - self.LSB
        self.lock.release()

    def makeEffect(self, level):
        # Level bounded by -1 and 1
        force = int(math.floor(level * 32767))
        envop = evdev.ff.Envelope(0, 0, 0, 0)
        const = evdev.ff.Constant(force, envop)
        effectType = evdev.ff.EffectType(ff_constant_effect=const)
        self.effect =  evdev.ff.Effect(evdev.ecodes.FF_CONSTANT, -1,
                                  0x6000, evdev.ff.Trigger(0,0),
                                  evdev.ff.Replay(int(self.dt*1000), 0), effectType)
    def runEffect(self):
        if self.effect_id != -1:
            self.dev.erase_effect(self.effect_id)
        self.effect_id = self.dev.upload_effect(self.effect)
        self.dev.write(evdev.ecodes.EV_FF, self.effect_id, 1)

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
