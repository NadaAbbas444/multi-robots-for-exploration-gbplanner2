#!/usr/bin/env python3
"""Make LARICS/ArduPilot-compatible copies of the gbplanner worlds.

The originals step physics at 4-10 ms (tuned for RotorS). ArduPilot SITL's
Gazebo plugin needs 1 ms steps at 1 kHz; at 250 Hz the kopterworx flight
controller oscillates and the UAV eventually gets thrown around. Everything
except the <physics> block is copied unchanged.
Usage:  python3 make_1khz_worlds.py   (run from this folder)
"""
import os, re
SRC = ["darpa_cave_01", "darpa_cave_02", "virginia_mine", "pittsburgh_mine", "edgar_mine"]
PHYSICS = """<physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>"""
here = os.path.dirname(os.path.abspath(__file__))
for name in SRC:
    s = open(os.path.join(here, "..", name + ".world")).read()
    s, n = re.subn(r"<physics\b.*?</physics>", PHYSICS, s, count=1, flags=re.S)
    if n == 0:
        s = s.replace("<world name=", "<world name=", 1)
        s = re.sub(r"(<world [^>]*>)", r"\1\n    " + PHYSICS, s, count=1)
    open(os.path.join(here, name + ".world"), "w").write(s)
    print("wrote", name + ".world")
