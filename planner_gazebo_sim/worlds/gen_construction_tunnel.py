#!/usr/bin/env python3
# Generates construction_tunnel.world:  python3 gen_construction_tunnel.py construction_tunnel.world
import sys
out = sys.argv[1]
X0, X1, HW, H = -10.0, 100.0, 7.0, 9.0      # tunnel x-range, half width, height
T = 0.3                                      # wall thickness
links = []
def box(name, cx, cy, cz, sx, sy, sz, color="0.45 0.43 0.40 1"):
    links.append(f"""
      <link name="{name}">
        <pose>{cx} {cy} {cz} 0 0 0</pose>
        <collision name="c"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
      </link>""")
L = X1 - X0; xc = (X0 + X1) / 2
box("floor", xc, 0, -T/2, L, 2*HW, T, "0.30 0.29 0.27 1")
box("ceiling", xc, 0, H + T/2, L, 2*HW, T, "0.35 0.35 0.35 1")
box("wall_left", xc, -HW - T/2, H/2, L, T, H)
# right wall with a side niche (x 82..94, depth 6 m)
NX0, NX1, ND = 82.0, 94.0, 6.0
box("wall_right_a", (X0+NX0)/2, HW + T/2, H/2, NX0 - X0, T, H)
box("wall_right_b", (NX1+X1)/2, HW + T/2, H/2, X1 - NX1, T, H)
box("niche_back", (NX0+NX1)/2, HW + ND + T/2, H/2, NX1 - NX0, T, H)
box("niche_side_a", NX0 - T/2, HW + ND/2, H/2, T, ND, H)
box("niche_side_b", NX1 + T/2, HW + ND/2, H/2, T, ND, H)
box("niche_floor", (NX0+NX1)/2, HW + ND/2, -T/2, NX1-NX0, ND, T, "0.30 0.29 0.27 1")
box("niche_ceiling", (NX0+NX1)/2, HW + ND/2, H + T/2, NX1-NX0, ND, T, "0.35 0.35 0.35 1")
box("end_start", X0 - T/2, 0, H/2, T, 2*HW, H)
box("end_far", X1 + T/2, 0, H/2, T, 2*HW, H)
# unfinished segment: a partially built arch/support frame and a scaffold block
for i, x in enumerate([20.0, 50.0, 70.0]):
    box(f"rib_{i}_l", x, -HW + 0.4, H/2, 0.6, 0.8, H, "0.55 0.55 0.55 1")
    box(f"rib_{i}_r", x,  HW - 0.4, H/2, 0.6, 0.8, H, "0.55 0.55 0.55 1")
box("scaffold", 88.0, HW + 3.0, 2.0, 4.0, 2.0, 4.0, "0.6 0.5 0.2 1")

tunnel = f"""
    <model name="construction_tunnel">
      <static>true</static>{''.join(links)}
    </model>"""

# vehicles = semantic objects of interest (ground-truth poses for evaluation)
vehicles = [
    ("bus",        "bus_1",        30.0, -3.5, 0.0,  1.5708),
    ("pickup",     "pickup_1",     44.0,  3.3, 0.0,  0.35),
    ("fire_truck", "truck_1",      60.0,  3.0, 0.0, -1.5708),
    ("suv",        "suv_1",        76.0, -3.0, 0.0,  2.0),
    ("pickup",     "pickup_2",     88.0,  9.8, 0.0,  0.0),   # inside the niche
]
clutter = [
    ("construction_cone",   "cone_1", 12.0,  1.5), ("construction_cone", "cone_2", 12.0, -1.5),
    ("construction_cone",   "cone_3", 38.0,  0.5), ("construction_barrel", "barrel_1", 24.0, 5.5),
    ("construction_barrel", "barrel_2", 66.0, -5.5), ("construction_barrel", "barrel_3", 96.0, 0.0),
    ("jersey_barrier", "barrier_1", 52.0, -5.8), ("jersey_barrier", "barrier_2", 80.0, 5.8),
]
inc = []
for uri, name, x, y, z, yaw in vehicles:
    inc.append(f"""
    <include><uri>model://{uri}</uri><name>{name}</name><static>true</static>
      <pose>{x} {y} {z} 0 0 {yaw}</pose></include>""")
for uri, name, x, y in clutter:
    inc.append(f"""
    <include><uri>model://{uri}</uri><name>{name}</name><static>true</static>
      <pose>{x} {y} 0 0 0 0</pose></include>""")

# uneven lighting along the tunnel (some segments dim, as on a construction site)
lights = []
for i, (x, inten) in enumerate([(0, 0.9), (15, 0.8), (30, 1.0), (45, 0.45), (60, 0.9), (75, 0.35), (88, 0.7)]):
    lights.append(f"""
    <light type="point" name="lamp_{i}">
      <pose>{x} 0 {H - 0.5} 0 0 0</pose>
      <diffuse>{inten} {inten*0.95} {inten*0.85} 1</diffuse><specular>0.1 0.1 0.1 1</specular>
      <attenuation><range>30</range><constant>0.4</constant><linear>0.03</linear><quadratic>0.001</quadratic></attenuation>
      <cast_shadows>false</cast_shadows>
    </light>""")

world = f"""<?xml version="1.0"?>
<!-- Under-construction tunnel with parked vehicles for the semantic inspection
     experiments (ICUAS 2025, Sec. V-B/C). 110 m x 14 m x 9 m, side niche at
     x=82..94. Vehicles (objects of interest), ground truth:
{chr(10).join(f'       {n:9s} {u:11s} x={x:6.1f} y={y:5.1f} yaw={yaw:5.2f}' for u,n,x,y,z,yaw in vehicles)}
     Vehicle/construction models: osrf/gazebo_models (CC-BY 3.0). -->
<sdf version="1.6">
  <world name="construction_tunnel">
    <physics type="ode"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate></physics>
    <scene><ambient>0.35 0.35 0.35 1</ambient><background>0.1 0.1 0.1 1</background><shadows>false</shadows></scene>
    <gravity>0 0 -9.81</gravity>
    <spherical_coordinates><surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>45.8</latitude_deg><longitude_deg>15.97</longitude_deg><elevation>0</elevation><heading_deg>0</heading_deg>
    </spherical_coordinates>{tunnel}{''.join(inc)}{''.join(lights)}
  </world>
</sdf>
"""
open(out, "w").write(world)
print("wrote", out, len(world), "bytes")
