"""
A PyrobotSimulator world. A large room with one robot and
many lights.

(c) 2005, PyroRobotics.org. Licensed under the GNU GPL.
"""

from pyrobot.simulators.pysim import TkSimulator, TkPioneer, \
     PioneerFrontLightSensors, PioneerFrontSonars

def INIT():
    # (width, height), (offset x, offset y), scale:
    sim = TkSimulator((441,434), (22,420), 40.357554)  
    # x1, y1, x2, y2 in meters:
    sim.addBox(0, 0, 10, 10)
    # (x, y) meters, brightness usually 1 (1 meter radius):
    sim.addLight(1, 1, 0.5)
    sim.addLight(2, 2, 0.5)
    sim.addLight(3, 3, 0.5)
    sim.addLight(4, 4, 0.5)
    sim.addLight(5, 5, 0.5)
    sim.addLight(6, 6, 0.5)
    sim.addLight(7, 7, 0.5)
    sim.addLight(8, 8, 0.5)
    sim.addLight(9, 9, 0.5)
    sim.addLight(10, 10, 0.5)
    # port, name, x, y, th, bounding Xs, bounding Ys, color
    # (optional TK color name):
    sim.addRobot(60000, TkPioneer("redRobot",
                                  5, 7, -0.86,
                                  ((.225, .225, -.225, -.225),
                                   (.175, -.175, -.175, .175)),
                                  "red"))
    sim.addRobot(60001, TkPioneer("blueRobot",
                                  6, 7, -0.86,
                                  ((.225, .225, -.225, -.225),
                                   (.175, -.175, -.175, .175)),
                                  "blue"))
    # add some sensors:
    sim.robots[0].addDevice(PioneerFrontLightSensors())
    sim.robots[1].addDevice(PioneerFrontLightSensors())
    sim.robots[1].addDevice(PioneerFrontSonars())
    return sim
