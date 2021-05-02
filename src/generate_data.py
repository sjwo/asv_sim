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
import rospy
from math import sin, cos, pi
from asv_sim.dynamics import Dynamics
from asv_sim.cw4 import cw4
from asv_sim import geodesic


class Generator():
    def __init__(self, verbosity=0, no_jitter=False):
        self.verbosity = verbosity
        self.boat = Dynamics(cw4)
        if no_jitter:
            self.debug("Setting zero jitter")
            for key in self.boat.jitters.keys():
                self.boat.jitters[key] = 0.0
        self.raw_data = list()
        self.diagnostics = list()
        self.observations = list()

    def debug(self, msg=str, level=1):
        if level <= self.verbosity:
            print("DEBUG: " + msg) 

    def do_warm_up(self):
        """Gets ASV to steady state. To be used before submitting controls from input file.

        MUST BE CALLED AFTER self.step IS SET.

        returns time at end of warmup

        cw4 takes 1 second to increase 1000 RPM; max RPM is 3200; start rpm is 0."""
        assert(self.step)
        time = rospy.Time()
        warmup_checkpoint = time + rospy.Duration.from_sec(3.3)
        while time < warmup_checkpoint:
            self.boat.update(1.0, 0.0, time)
            time += self.step
        return time

    def generate(self, filename, repetitions=1):
        """
        Reads control file, runs simulation accordingly, and returns data.

        :param filename: for file containing control inputs
        """
        with open(filename, 'r') as control:
            self.period = rospy.Duration.from_sec(float(control.readline().strip()))
            # convert input milliseconds to Duration nanoseconds
            self.step = rospy.Duration(nsecs=int(control.readline().strip()) * 1000000)
            self.debug("self.period: {}, self.step: {}".format(self.period.to_sec(), self.step.to_sec()), 3)
            assert(self.step < self.period)

            # warm up asv to steady state
            time = self.do_warm_up()
            checkpoint = time + self.period

            # collect data using controls from input file
            for line in control:
                self.debug("new control {} at {} with checkpoint {}".format(line.strip(), time.to_sec(), checkpoint.to_sec()))
                for _rep in range(repetitions):
                    while time < checkpoint:
                        (throttle, rudder) = line.strip().split()
                        throttle = float(throttle)
                        rudder = float(rudder)
                        self.debug("  {} {} @ {}".format(throttle, rudder, time.to_sec()), level=2)
                        diagnostics = self.boat.update(throttle, rudder, time)
                        self.raw_data.append(
                            {
                                'time': time.to_sec(),
                                'throttle': throttle,
                                'rudder': rudder,
                                'speed': self.boat.speed,
                                'lat': self.boat.latitude,
                                'lon': self.boat.longitude,
                                'heading': self.boat.heading,
                                'cog': self.boat.cog,
                                'sog': self.boat.sog,
                            }
                        )
                        self.diagnostics.append(diagnostics)
                        time += self.step
                    checkpoint += self.period

    @staticmethod
    def modrad(theta):
        sign = 1
        if theta < 0:
            sign = -1
            theta *= sign
        theta = theta % (2 * pi)
        return theta * sign

    @staticmethod
    def calc_heading_to_azimuth(heading, azimuth):
        """Finds difference in radians between arguments.

        Arguments are in (nonnegative?) radians clockwise from north.

        Returns difference in range [-pi, pi].
        """
        # modulo 2pi
        heading = Generator.modrad(heading)
        azimuth = Generator.modrad(azimuth)

        # find difference
        theta = azimuth - heading

        # find equivalent difference that is less that pi
        if theta < -pi:
            theta_norm = 2 * pi + theta
        elif theta > pi:
            theta_norm = 2 * pi - theta
        else:
            theta_norm = theta
        return theta_norm

    @staticmethod
    def raw_to_obs_helper(start, end):
        """start and end are each a datapoint (element) from the self.raw_data list of (...tuples of?...) dictionaries
        """
        azimuth, distance = geodesic.inverse(
            start['lon'],
            start['lat'],
            end['lon'],
            end['lat'],
        )

        # TODO double check this. Do I need to modulo or normalize?
        yaw = end['heading'] - start['heading'] 

        theta = Generator.calc_heading_to_azimuth(start['heading'], azimuth)
        surge = distance * cos(theta)
        sway = distance * sin(theta)
        return (surge, sway, yaw)

    def convert_to_observations(self):
        """
        MUST BE CALLED AFTER generate()
        """
        n_steps_per_observation = int(self.period.to_sec() / self.step.to_sec())
        data_ix = 0
        # print(len(self.raw_data))
        while data_ix < len(self.raw_data):
            start = self.raw_data[data_ix]
            end = self.raw_data[data_ix + n_steps_per_observation - 1]
            self.debug("{} start: \n{}".format(data_ix, start), level=3)
            self.debug("{} end: \n{}".format(data_ix, end), level=3)
            data_ix += n_steps_per_observation
            surge, sway, yaw = self.raw_to_obs_helper(start, end)
            self.observations.append(
                {
                    'throttle': start['throttle'],
                    'rudder': start['rudder'],
                    'surge': surge,
                    'sway': sway,
                    'yaw': yaw
                }
            )

    def print_raw_data(self):
        for data_point in self.raw_data:
            print(data_point)

    def print_diagnostics(self):
        for step in self.diagnostics:
            print("throttle: {}, rpm: {}, prop_rpm: {},".format(
                step['throttle'],
                step['rpm'],
                step['prop_rpm']
                ))

    def print_observations(self):
        # TODO after figuring out units in raw_to_obs_helper, figure out how I want to print them here.
        print("throttle, rudder, surge, sway, yaw")
        for obs in self.observations:
            print("{}, {}, {}, {}, {}".format(
                obs['throttle'],
                obs['rudder'],
                obs['surge'],
                obs['sway'],
                obs['yaw'],
            ))

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("control_filename", type=str)
    parser.add_argument('--verbose', '-v', action="count")
    parser.add_argument('--no-jitter', default=False, action="store_true")
    parser.add_argument('--repetitions', type=int, default=1, help='number of periods to repeat each control before executing next control')
    args = parser.parse_args()
    gen = Generator(verbosity=args.verbose, no_jitter=args.no_jitter)
    gen.generate(args.control_filename, repetitions=args.repetitions)
    if args.verbose > 1:
        gen.print_raw_data()
        if args.verbose > 2:
            gen.print_diagnostics()
    gen.convert_to_observations()
    gen.print_observations()

def p(factor):
    return factor * pi

def f(theta):
    return theta / pi

def modrad(theta):
    sign = 1
    if theta < 0:
        sign = -1
        theta *= sign
    theta = theta % (2 * pi)
    return theta * sign

def test():
    g = Generator()
    for heading_f, azimuth_f in [
            (0, 0.25),
            (0.25, 0),
            (1.6, 0.4),
            (1.4, 0.6),
            (0.0, 0.0),
            (2, 0),
            (0, 2),
            (2.1, 0.2),
            (0.2, 2.1),
        ]:
        heading = p(heading_f)
        azimuth = p(azimuth_f)

        # modulo 2pi
        heading = modrad(heading)
        azimuth = modrad(azimuth)

        # find difference
        theta = azimuth - heading

        # find equivalent difference that is less that pi
        if theta < -pi:
            theta_norm = 2 * pi + theta
        elif theta > pi:
            theta_norm = 2 * pi - theta
        else:
            theta_norm = theta
        print("heading: {}, azimuth: {}, theta: {}, theta_norm: {}".format(
            f(heading), f(azimuth), f(theta), f(theta_norm)
            ))

if __name__ == '__main__':
    main()
    # test()
