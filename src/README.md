# README: generate_data.py

## control file syntax

Each line represents a control configuration to be held constant for DURATION seconds. During DURATION, the control will be submitted to the simulator at FREQUENCY (Hz, or submissions per second), with appropriate time stamps.

Valid rudder ranges: -1.0 to 1.0

Valid throttle ranges: 0.0 to 1.0

    [duration (seconds)]
    [frequency (Hz)]
    [[throttle] [rudder]]
    [[throttle] [rudder]]
    [[throttle] [rudder]]
    [[throttle] [rudder]]
    [[throttle] [rudder]]
    [[throttle] [rudder]]
    ...    