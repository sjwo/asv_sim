# README: generate_data.py

## control file syntax

Each line represents a control configuration to be held constant for PERIOD seconds. During PERIOD, the control will be submitted to the simulator every STEP milliseconds, with appropriate time stamps.

Valid rudder ranges: -1.0 to 1.0

Valid throttle ranges: 0.0 to 1.0

    [period]
    [step]
    [[throttle] [rudder]]
    [[throttle] [rudder]]
    [[throttle] [rudder]]
    [[throttle] [rudder]]
    [[throttle] [rudder]]
    [[throttle] [rudder]]
    ...    