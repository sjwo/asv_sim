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

def main():
    mine = Dynamics(cw4)

if __name__ == '__main__':
    main()