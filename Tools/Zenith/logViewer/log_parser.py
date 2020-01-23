#!/usr/bin/env python
import os
import sys
from os.path import dirname
import subprocess
import json
from scipy import io as scipyio

ROOT_DIR = os.path.join(dirname(__file__), "../../..")
MAVLOG_PATH = os.path.join(ROOT_DIR, "modules/mavlink/pymavlink/tools/mavlogdump.py")

LOG_PATH = (
    "/Users/bbevillard/Documents/Zenith/ardupilot/Tools/Zenith/sim/logs/00000002.BIN"
)

if __name__ == "__main__":
    print("Parsing logs... ")
    cmd = [sys.executable, MAVLOG_PATH, "--planner", "--format", "json", LOG_PATH]
    out = subprocess.check_output(cmd).decode("utf-8")
    logs = {}
    start_time = None
    for l in out.split("\n"):
        if not l:
            continue
        msg = json.loads(l)
        msg_type = msg["meta"]["type"]
        msg_timestamp = msg["meta"]["timestamp"]
        if not start_time:
            start_time = msg_timestamp
        msg_timestamp -= start_time
        msg_data = msg["data"]
        if msg_type not in logs:
            logs[msg_type] = {}
        type_dict = logs[msg_type]
        msg_data["timestamp"] = msg_timestamp
        for k, v in msg_data.items():
            if k not in type_dict:
                type_dict[k] = []
            type_dict[k].append(v)
    scipyio.savemat("log.mat", logs)
    print("Done")
