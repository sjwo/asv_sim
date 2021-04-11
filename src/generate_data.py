#!/usr/bin/env python

"""
Generate Data

Stephen Wissow
sjw1000@wildcats.unh.edu
Spring 2021

Generates simulated ASV (CW4) data for use with UNH CS 850 S21 ML Project
re forward model of boat.

Python 2, please (just because I'm working on machine with Melodic ROS, which uses Python 2 so the rospy import assumes Python 2).
"""

import argparse
from asv_sim.dynamics import Dynamics
from asv_sim.cw4 import cw4
import rospy

class Generator():
    def __init__(self, verbosity=0):
        self.boat = Dynamics(cw4)
        self.verbosity = verbosity

    def debug(self, msg=str, level=1):
        if level <= self.verbosity:
            print("DEBUG: " + msg) 

    def generate(self, filename):
        """
        Reads control file, runs simulation accordingly, and returns data.

        :param filename: for file containing control inputs
        """
        with open(filename, 'r') as control:
            period = rospy.Duration.from_sec(float(control.readline().strip()))
            # convert input milliseconds to Duration nanoseconds
            step = rospy.Duration(nsecs=int(control.readline().strip()) * 1000000)
            self.debug("period: {}, step: {}".format(period.to_sec(), step.to_sec()), 3)
            assert(step < period)
            time = rospy.Time()
            checkpoint = time + period
            for line in control:
                self.debug("new control {} at {} with checkpoint {}".format(line.strip(), time.to_sec(), checkpoint.to_sec()))
                while time < checkpoint:
                    (throttle, rudder) = line.strip().split()
                    throttle = float(throttle)
                    rudder = float(rudder)
                    self.debug("  {} {} @ {}".format(throttle, rudder, time.to_sec()), level=2)
                    self.boat.update(throttle, rudder, time)
                    time += step
                checkpoint += period

"""
basically, I'll start a Timer, pass it my own callback.

my own callback will grab the TimerEvent instance that it's passed,
will extract the time and pass that to Dynamics.update() and pass the next set of controls.

I probably want to create my own object that eats an input file of commands,
and then steps through them each time its callback is passed.

I'll need to capture the data somehow, too..maybe I'll be subscribing to topics? So, there's no ROS node running...I think I'll actually just be checking the state of the Dynamics object, and recording that.

TODO:
turn all errors, disturbances, current to zero

QUESTION: at what frequency do I need to send the constant (repeated) control? Is that 10 Hz? Maybe just parameterize?
"""

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("control_filename", type=str)
    parser.add_argument('--verbose', '-v', action="count")
    args = parser.parse_args()
    gen = Generator(args.verbose)
    gen.generate(args.control_filename)

if __name__ == '__main__':
    main()