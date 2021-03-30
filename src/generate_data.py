#!/usr/bin/env python

"""
Generate Data

Stephen Wissow
sjw1000@wildcats.unh.edu
Spring 2021

Generates simulated ASV (CW4) data for use with UNH CS 850 S21 ML Project
re forward model of boat.

Python 3, please.
"""

from asv_sim.dynamics import Dynamics
from asv_sim.cw4 import cw4

class Generator():
    def __init__(self):
        self.boat = Dynamics(cw4)

    def generate(self, filename):
        """
        Reads control file, runs simulation accordingly, and returns data.

        :param filename: for file containing control inputs
        """
        pass

    def next_control(self)
# TODO create my own callback function
# TODO import rospy.Timer
"""
basically, I'll start a Timer, pass it my own callback.

my own callback will grab the TimerEvent instance that it's passed,
will extract the time and pass that to Dynamics.update() and pass the next set of controls.

I probably want to create my own object that eats an input file of commands,
and then steps through them each time its callback is passed.

I'll need to capture the data somehow, too..maybe I'll be subscribing to topics? So, there's no ROS node running...I think I'll actually just be checking the state of the Dynamics object, and recording that.

TODO:
turn all errors, disturbances, current to zero


"""

def main():
    print("Hello, world!")
    pass

if __name__ == '__main__':
    main()