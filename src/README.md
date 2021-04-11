# README: generate_data.py

## control file syntax

Each line represents a control configuration to be held constant for PERIOD seconds. During PERIOD, the control will be submitted to the simulator every STEP milliseconds, with appropriate time stamps.

Valid rudder ranges: -1.0 to 1.0

Valid throttle ranges: 0.0 to 1.0

Generator makes boat warm up to full 3200 rpm, which takes 3.3 seconds. Recommended to issue full throttle of 1 for each control line in order to maintain constant velocity.

    [period]
    [step]
    [[throttle] [rudder]]
    [[throttle] [rudder]]
    [[throttle] [rudder]]
    [[throttle] [rudder]]
    [[throttle] [rudder]]
    [[throttle] [rudder]]
    ...    