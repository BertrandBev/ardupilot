#!/usr/bin/env python

import argparse
import os
import json
import subprocess
import shutil

ARDUPILOT_PATH = os.environ["ARDUPILOT_PATH"]
SIM_SCRIPT = "/Tools/autotest/sim_vehicle.py"
dir_path = os.path.dirname(os.path.realpath(__file__))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run Z1 simulation")
    parser.add_argument("--joystick", action="store_true", help="load joystick module")
    parser.add_argument("--fp", action="store_true", help="load flight plan")
    parser.add_argument("--speedup", action="store_true", help="enable speedup")
    parser.add_argument("--clear", action="store_true", help="clear sim data")
    args = parser.parse_args()

    # Clear data if needed
    if args.clear:
        shutil.rmtree(os.path.join(dir_path, "logs"))
        shutil.rmtree(os.path.join(dir_path, "terrain"))
        os.remove(os.path.join(dir_path, "eeprom.bin"))
        print('Sim data cleared')
        exit(-1)

    # Build sim command
    script = [ARDUPILOT_PATH + SIM_SCRIPT]
    sim_args = [
        "-v",
        "ArduPlane",
        "-f",
        "Z1",
        "--no-rebuild",
        # "--aircraft",
        # "ArduPlane",
        # "--wipe-eeprom",
    ]
    if args.joystick:
        sim_args += ["--joystick"]
    if args.speedup:
        sim_args += ["--speedup", "10"]
    mav_arg_list = [
        "--logfile logs/flight.tlog",
        '--cmd-imu-ready "wp load /Users/bbevillard/Documents/Zenith/ardupilot/Tools/Zenith/flightplans/circuit.txt"',
        '--cmd-fp-ready "mode auto; arm throttle"',
    ]
    mav_args = ["--mavproxy-args", json.dumps(" ".join(mav_arg_list))] if args.fp else []
    # print(" ".join(mav_args)); exit(-1)
    cmd = script + sim_args + mav_args

    os.system(" ".join(cmd))

    # Execute sim command. TODO: run in subprocess
    # p = subprocess.Popen(cmd, cwd=working_directory)
    # p.wait()
