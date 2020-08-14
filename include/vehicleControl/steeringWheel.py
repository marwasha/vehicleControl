import pygame
import numpy as np
import evdev
import math
pygame.init()
pygame.joystick.init()

class Wheel:
    def __init__(self, channel, dev):
        assert pygame.joystick.get_count() != 0, 'No joystick'
        self.js = pygame.joystick.Joystick(channel)
        self.js.init()
        self.dev = evdev.InputDevice(dev)
        self.dt = .1
        self.effect_id = -1
        self.update()

    def update(self):
        pygame.event.get()
        self.angle = self.js.get_axis(0) #*450/180*np.pi
        self.throttle = -(self.js.get_axis(1)-1)/2
        self.brake = -(self.js.get_axis(2)-1)/2
        self.A = self.js.get_button(0)
        self.B = self.js.get_button(1)
        self.X = self.js.get_button(2)
        self.Y = self.js.get_button(3)

    def makeEffect(self, level):
        # Level bounded by -1 and 1
        force = int(math.floor(level * 32767))
        envop = evdev.ff.Envelope(0, 0, 0, 0) # From example
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
        out = "Angle:{} Gas:{} Break:{} \n".format(
                self.angle, self.throttle, self.brake)
        out += "A:{} B:{} X:{} Y:{}".format(
                self.A, self.B, self.X, self.Y)
        return out
