# Graph-Based Decentralized Exploration and Semantic Inspection for Multi-Robot Systems

Code for the ICUAS 2025 paper by **Nada E. Abbas** and **Tamara Petrović**
(LARICS, University of Zagreb FER).

A team of UAVs explores an unknown subterranean or large-scale environment,
detects objects of interest, and inspects them, with no central controller:

- **Graph-based exploration**: a dense local RRG for immediate exploration and
  a sparse global graph for repositioning, dead-end recovery and auto-homing,
  inside an adaptive local bounding box (built on gbplanner).
- **Peer-to-peer coordination with No-Go Zones**: each UAV shares the goals it
  has selected as No-Go Zones. Peers may fly through them, but frontiers inside
  them get zero gain and are never selected, which prevents redundant coverage.
- **Semantic inspection**: Mask R-CNN detects objects (vehicles). The
  inspection planner clusters unmapped holes of the object's surface in the
  TSDF, samples a viewpoint graph around it, and orders the viewpoints with
  the Lin-Kernighan-Helsgaun (LKH-3) TSP heuristic.
- **Dynamic task allocation**: a sequential auction or fixed explorer/inspector
  roles, for any number of UAVs; tasks detected later are allocated too.

Project page: https://nadaabbas444.github.io/multi-robots-for-exploration-gbplanner2/

---

## Quick start

Everything runs in Docker (Ubuntu 20.04, ROS Noetic, Gazebo 11, ArduPilot SITL
via the [LARICS UAV simulation stack](https://github.com/larics/uav_ros_simulation)).
The host only needs Docker and an X server. About 20 GB of disk is needed.

```bash
# 1. workspace + sources (dependencies are pinned, see workspace/deps.repos)
mkdir -p ~/icuas2025_ws/src && cd ~/icuas2025_ws/src
git clone https://github.com/NadaAbbas444/multi-robots-for-exploration-gbplanner2.git
./multi-robots-for-exploration-gbplanner2/workspace/setup_workspace.sh

# 2. container (the image is built on first use, which takes ~15 min)
./multi-robots-for-exploration-gbplanner2/workspace/docker/run.sh
```

Inside the container:

```bash
build_ws                 # first time and after code changes (~10 min the first time)
start_sim                # 2 UAVs in the DARPA cave, auction allocation
start_exploration        # once both UAVs hover (~2 min): start autonomous exploration
tmux attach -t icuas2025 # watch all processes (Ctrl-b d to detach)
stop_sim
```

`docker/run.sh` with no arguments opens a shell. Run it again in another host
terminal to get a second shell in the same container.

## Scenarios

`start_sim` and `run_test` are configured with environment variables:

| Variable   | Values                          | Default   |
|------------|---------------------------------|-----------|
| `SCENARIO` | `cave` · `tunnel` · `mine`      | `cave`    |
| `NUM_UAVS` | `2` (red, blue) · `3` (+yellow) | `2`       |
| `STRATEGY` | `auction` · `role`              | `auction` |
| `SEMANTIC` | `true` · `false` (Mask R-CNN)   | `true` in `tunnel`, else `false` |
| `GUI`      | `true` · `false` (Gazebo + RViz)| `false`   |
| `WORLD`    | path to any `.world`            | from `SCENARIO` |

```bash
GUI=true start_sim                                  # watch exploration in Gazebo/RViz
SCENARIO=tunnel start_sim                           # semantic inspection with auction allocation
SCENARIO=tunnel STRATEGY=role start_sim             # same, role-based allocation
NUM_UAVS=3 SCENARIO=tunnel start_sim                # three UAVs
```

Worlds:

- `cave`: DARPA SubT cave (`planner_gazebo_sim/worlds/larics/darpa_cave_01.world`).
- `mine`: room-and-pillar mine (`.../larics/virginia_mine.world`).
- `tunnel`: tunnel under construction, 110 m × 14 m × 9 m, with a bus, a
  fire truck, two pickups and an SUV
  (`planner_gazebo_sim/worlds/construction_tunnel.world`; ground-truth poses are
  listed in its header). Regenerate it with `worlds/gen_construction_tunnel.py`.

The `worlds/larics/` copies are the original gbplanner worlds with 1 kHz
physics. ArduPilot SITL becomes unstable at the originals' 4–10 ms steps.

### Headless test runs

```bash
run_test 10                       # 10 min, prints both UAVs' state every minute
NUM_UAVS=3 SCENARIO=tunnel run_test 6
```

Results (summary, planner log, rosbag) go to
`workspace/sim/icuas2025_larics/results/<timestamp>/`.

### Task allocation and inspection without the detector

```bash
inject_object 30 -3.5 1.5 red     # report an object at (x, y, z) as seen by red
```

After `exploration_time_budget` (default 500 s), the allocator auctions all
known objects. The winner pauses exploration, flies to the object, inspects
it along the LKH tour, reports `done_inspection` and resumes exploring.

## Configuration

`gbplanner/config/larics/` contains the configuration used with the LARICS drone:

- `gbplanner_config.yaml`: planner parameters (sensor model matches the
  simulated 8-beam ±30° LiDAR).
- `task_allocation.yaml`: strategy, roles, exploration budget `T_e`, merge
  radius, inspection time, battery budget.
- No-Go Zones, as private parameters of `gbplanner_node`:
  `no_go_zones/{enable, radius, min_spacing, apply_own_zones}` and
  `peer_namespaces`.
- `use_central_frontier_assigner` (default `false`) re-enables the optional
  central `/assign_frontier` node (`p2p`).

## Repository layout

| Path | Content |
|------|---------|
| `gbplanner/` | planner: local/global RRG, No-Go Zones (`rrg.cpp`), inspection state machine, LKH wrapper (`lkh_tsp.cpp`), task allocation (`Dynamic_Auction.cpp`), LARICS bridge nodes |
| `gbplanner/launch/larics/` | `robot.launch` (one UAV), `icuas2025_two_uav.launch`, `icuas2025_three_uav.launch` |
| `mask_rcnn_ros/` | Mask R-CNN detector (TF 2.4) and object localisation |
| `planner_common/`, `planner_msgs/`, `planner_semantic_msgs/`, ... | gbplanner libraries and messages |
| `planner_gazebo_sim/` | worlds and models |
| `workspace/` | Docker images, simulation sessions, helper commands, pinned dependencies, patches |

## Citation

```bibtex
@inproceedings{abbas2025graph,
  title     = {Graph-Based Decentralized Exploration and Semantic Inspection for Multi-Robot Systems},
  author    = {Abbas, Nada E. and Petrovi{\'c}, Tamara},
  booktitle = {2025 International Conference on Unmanned Aircraft Systems (ICUAS)},
  year      = {2025}
}
```

## Acknowledgements and licenses

- Built on [gbplanner_ros](https://github.com/ntnu-arl/gbplanner_ros)
  (BSD-3-Clause, see `LICENSE` and `README_gbplanner.md`) and its dependencies
  (voxblox, adaptive_obb_ros, pci_general, ...).
- Simulation: [LARICS uav_ros_simulation](https://github.com/larics/uav_ros_simulation).
- Detection: [mask_rcnn_ros](https://github.com/ethz-asl/mask_rcnn_ros) /
  [Matterport Mask R-CNN](https://github.com/matterport/Mask_RCNN) (MIT).
- TSP: [LKH-3](http://webhotel4.ruc.dk/~keld/research/LKH-3/) by K. Helsgaun
  (research use only), built in the Docker image from a source mirror.
- Vehicle and construction models: [osrf/gazebo_models](https://github.com/osrf/gazebo_models)
  (CC-BY 3.0, see `planner_gazebo_sim/models/GAZEBO_MODELS_NOTICE.md`).
