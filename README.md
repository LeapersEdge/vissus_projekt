# vissus_projekt
## Project: 2nd phase
### Controlling a Swarm of Crazyflies using Consensus Protocol and Auction-Based Task Allocation

  Group: Gabrijel Biočić, Dino Dubinović, Quinn Lee Fletcher, Ita Poklepović, Antonio Škara.

Dependencies (links to installation guides):

- ROS2: Humble

- [ROS2 stage](https://github.com/tuw-robotics/stage_ros2/blob/humble/res/install.md) [(alternative if the previous instruction doesn't work)](https://github.com/LeapersEdge/vissus_projekt/blob/main/docs/stage%20installation.md)

- [All from MRS instructions](https://github.com/larics/mrs_simulation/blob/main/README.md#2-manual-installation-if-you-already-have-ros-installed)

Build:

```shell
colcon build --merge-install --symlink-install --packages-select vissus_projekt
```

### Concept

#### Nodes
- `boid-control`: active for each individual robot, subscribes to its own topic in which there are necessary odometry msgs for each robot in the system 
- `data-splitter`: only one is launched and publishes all topics relevant for each robot
- `auctioneer`: actions tasks between bidders
- `bidder`: bid on the task (each bidder represents one robot)

Boid controller takes 2 parameters: 
- ID of the robot it is representing (default: 0)
- Mode of operation / Working mode (default: "market")

#### Messages
- `OdometryArray`
- `TuningParams`
- `Bid`
- `Formation`
- `Task`
- `TaskList`

#### Boid control
Every robot uses ROS2 topics of which it subscribes and publishes to the following:
- Subscribers:
  - `/adjacency_matrix` (determines which robots' positions should be considered when calculating movement)
  - `/robot_<self_ID>/boid_info` (all currently available and necessary information about robots enviroment)
  - `/tuning_params` (state-independent parameters that determine robots' behaviour)
  - `/robot_goals` (goal to which the robot is trying to get)
  - `/formation` (information about the desired formation)
- Publishers:
  - `/cf_<self_ID>/cmd_vel` (velocity command)

#### Data splitter
Instead of having one node that manages everyone (centralized management), we **do decentralized management** (each boid manages itself).

Since each individual boid does not know by itself where every other boid is, we use a `data-splitter` node, where the message of **its odometry** and **the odometry of each of the boids it sees** will be constructed.

`data-splitter`, therefore, takes all boids' odometries and produces an odmetry array from them, which it sends to each individual boid. It is essentially an information center that sends each robot the information it needs to make decisions.

![Graph node za primjer s 6 boida](https://github.com/LeapersEdge/vissus_projekt/blob/main/images/node_graph3.jpeg)

### How to use

Set up the drone's directed communication graph in `launch/topology`.
To read up on how to do that, open the `launch/topology` and read the comments at the end of the file.

Depending on whether you want to use the market or just a simple version, the following commands will be used:
Market:
```shell
cd ~/ros2_ws/src/vissus_projekt/launch
./market_start.sh
```

Simple version:
```shell
cd ~/ros2_ws/src/vissus_projekt/launch
./simple_start.sh
```

You need to set the system parameters in two places.
First in launch/launch_params.yaml you need to set:
1. Number of boids
2. FOV of the boids
3. Its visual range
4. Mode of operation / Working mode ('rendevous', 'market' or 'formation')

You set the system parameters by publishing a message of type TuningParams msg. We recommend `rqt` for this, but you can also easily do it with `ros2 topic pub`.

#### Parameters
It is advised to modify the parameters via rqt; change the parameters in `/tuning_params`. The parameters and what they control are listed below, along with some values found to be acceptable while testing.
- `rotation_kp`: rotation rate amplification, `1.0`
- `max_speed`: maximum speed of each boid, `1.0`
- `avoidance_range`: range at which to avoid running into other boids, `0.5`
- `avoidance_factor`: intensity with which to avoid running into other boids, `-0.3`
- `census_factor`: intensity of applied consensus rule, `0.4`
- `goal_factor`: intensity with which to move towards the goal, `0.1`

##### Rule of thumb for parameters
1. `avoidance_factor` should be sufficiently high; great enough to make sure no collisions are happening, but low enough to prevent violent and sudden jerking
2. `avoidance_range` should be sufficiently small; great enough to affect boids, but not too small to prevent violent and sudden jerking
3. `goal_factor` should be lower than consensus factor to allow boids to form it at a reasonable pace

### Experimental setup (real drones)
0.1. Build everything with `colcon build --symlink-install` and source.
0.2. Use `cfclient` to check if you can connect with the drone
1. Modify by need `mrs_crazyfile_exp/config/crazyflies_mrs.yaml`
2. Run nodes that interact with antenna and real drones
```bash
cd ~/ros2_ws/src/mrs_crazyfiles_exp/startup
./start.sh
```
3. Modify launch_params.yaml as needed, especially the  `num_of_robots` part.
4. Launch with one of the `_start.sh` scripts within `~/ros2_ws/src/vissus_projekt/launch` directory
