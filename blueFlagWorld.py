"""
A PyrobotSimulator world. A large room with one robot and
many lights.

(c) 2005, PyroRobotics.org. Licensed under the GNU GPL.
"""

from pyrobot.simulators.pysim import TkSimulator, TkPioneer, \
     PioneerFrontLightSensors, PioneerFrontSonars

from math import *

def INIT():
    # (width, height), (offset x, offset y), scale:
    sim = TkSimulator((441,434), (22,420), 40.357554)  
    # x1, y1, x2, y2 in meters:
    sim.addBox(0, 0, 10, 10, "green")
    # (x, y) meters, brightness usually 1 (1 meter radius):
    sim.addLight(5, 5, 0.3)
    # port, name, x, y, th, bounding Xs, bounding Ys, color
    # (optional TK color name):
    sim.addRobot(60002, TkPioneer("redRobot",
                                  5, 1, 0,
                                  ((.225, .225, -.225, -.225),
                                   (.175, -.175, -.175, .175)),
                                  "red"))
    sim.addRobot(60003, TkPioneer("blueRobot",
                                  5, 9, pi,
                                  ((.225, .225, -.225, -.225),
                                   (.175, -.175, -.175, .175)),
                                  "blue"))

    sim.addBox(4.8, 0.2, 5.2, 0.6, "black")
    sim.addBox(4.8, 9.4, 5.2, 9.8, "white")

    # add some sensors:
    sim.robots[0].addDevice(PioneerFrontLightSensors())
    sim.robots[0].addDevice(PioneerFrontSonars())
    sim.robots[1].addDevice(PioneerFrontLightSensors())
    sim.robots[1].addDevice(PioneerFrontSonars())
    return sim

