#!/usr/bin/env python

import argparse
import os
import json

ARDUPILOT_PATH = os.environ["ARDUPILOT_PATH"]
SIM_SCRIPT = "/Tools/autotest/sim_vehicle.py"

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run Z1 simulation")
    parser.add_argument("--joystick", action="store_true", help="load joystick module")
    parser.add_argument("--fp", action="store_true", help="load flight plan")
    parser.add_argument("--speedup", action="store_true", help="enable speedup")
    args = parser.parse_args()

    # Build sim command
    script = [ARDUPILOT_PATH + SIM_SCRIPT]
    sim_cmds = [
        "-v",
        "ArduPlane",
        "-f",
        "Z1",
        "--no-rebuild",
        "--aircraft",
        "ArduPlane",
        # "--wipe-eeprom",
    ]
    if args.joystick:
        sim_cmds += ["--joystick"]
    if args.speedup:
        sim_cmds += ["--speedup", "10"]
    mav_cmds = [
        '--cmd-imu-ready "wp load /Users/bbevillard/Documents/Zenith/ardupilot/Tools/Zenith/flightplans/circuit.txt"',
        '--cmd-fp-ready "mode auto; arm throttle"'
    ]
    mav_args = ["--mavproxy-args", json.dumps(" ".join(mav_cmds))] if args.fp else []
    # print(" ".join(mav_args)); exit(-1)
    cmd = script + sim_cmds + mav_args
    # Execute sim command. TODO: run in subprocess
    os.system(" ".join(cmd))
