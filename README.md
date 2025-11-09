# vissus_projekt

  Grupa: Gabrijel Biočić, Dino Dubinović, Quinn Lee Fletcher, Ita Poklepović, Antonio Škara.

Dependencies (links to installation guides):

- ROS2: Humble

- [ROS2 stage](https://github.com/tuw-robotics/stage_ros2/blob/humble/res/install.md) [(alternativa ako faila)](https://github.com/LeapersEdge/vissus_projekt/blob/main/docs/stage%20installation.md)

- [stvari iz uputa](https://github.com/larics/mrs_simulation/blob/main/README.md#2-manual-installation-if-you-already-have-ros-installed)

ROS2 nema catkin; koristi colcon:

```shell
colcon build
```

### Concept

#### Nodes
Imamo *node*ove:
- `boid-control`: aktivira se za svaki bot, *subscribe*a na svoj topic u kojem se nalaze odmetry msgs od svakog boida u svome susjedstvu, i nalazi pozu najbliže prepreke) 
- `data-splitter`: pokreće se samo jednom i *publish*ati će sve topic-ove koje za svakog boida

Controller ima jedan parametar: ID od robota na kojeg se *subscribe*ati.

#### Messages
- `OdometryArray`: header, odometries[], closest_obstacle
- `TuningParams`

#### Boid control
Svaki boid ima 2 *message*-a:
- `odom` (odometrija)
- `cmd_vel` (velocity command)

#### Data splitter
Umjesto da imamo jedan node koji upravlja svima (centralizirano upravljanje), mi **radimo decentralizirano upravljanje** (svaki boid upravlja sobom).

Pošto svaki pojedini boid ne zna sam po sebi gdje je svaki drugi boid, koristimo `data-splitter` node, gdje će se konstruirati message **svoje odometrije**, **odometrije svakog od boida kojeg vidi**, te **closest obstacle**.

`data-splitter`, dakle, uzme sve odometrije boidova i iz njih proizvede odmetry array kojeg šalje svakom pojedinačnom boidu. On je esencijalno centar informacije koji šalje svakom robotu informaciju potrebnu da napravi odluke.

![Graph node za primjer s 6 boida](https://github.com/LeapersEdge/vissus_projekt/blob/main/images/node_graph.png)

### How to use
Pokrenite sustav pokretanjem simulation.launch.py skripte
```shell
ros2 launch vissus_projekt simulation.launch.py
```
Parametre sustave morate podesiti na dva mjesta.
Prvo u launch/launch_params.yaml morate podesiti: 
  1. broj boida: 
  2. FOV boida
  3. Njegov vidni dosed

Parametre sustava(koji su zapravo pojačanja reynolodsovih pravila) namještate tako da publishate poruku tipa TuningParams msg. Mi preporučujemo rqt za to, ali možete i lagano iskoristiti ros2 topic pub.
```shell
rqt
```

#### Parameters
It is advised to modify the parameters via rqt; change the parameters in `/tuning_params`. The parameters and what they control are listed below, along with some values found to be acceptable while testing.
- `rotation_kp`: rotation rate amplification, `1`
- `cohesion_range`: radius in which to attempt cohesion, `3`
- `cohesion_factor`: strength with which cohesion is attempted, `5` (negative value for dispersion)
- `alignment_range`: radius in which to consider alignment partners, `2`
- `alignment_factor`: strength with which to maintain alignment, `10`
- `avoidance_range`: range at which to avoid running into other boids, `0.75`
- `avoidance_factor`: intensity with which to avoid running into other boids, `100`
- `obstacle_avoidance_range`: range within which to avoid obstacles & borders, `1`
- `obstacle_avoidance_factor`: intensity with which to avoid obstacles & borders, `100` (negative value for attraction)
- `goal_factor`: intensity with which to move towards the goal, `25`

##### Rule of thumb for parameters
1. `avoidance_factor` and `obstacle_avoidance_factor` should be very high; the highest of all parameters
2. `avoidance_range` and `obstacle_avoidance_range` should be very small; the lowest of all parameters, somewhere around `1` or less
3. `cohesion_range` should be equal to or slightly greater than `alignment_range` (i. e. 50% greater, `3` vs. `2`)
4. `cohesion_factor` should be equal to or lesser than `alignment_factor` (i. e. 50% smaller, `5` vs. `10`)
5. `goal_factor` should be greater than `cohesion_factor` and `alignment_factor`, but considerably lower than `avoidance_factor` and `obstacle_avoidance_factor`


#### To-do
  1. Testirati simulaciju i isprobati razne parametre, parametri su definirani u msg/TuningParams.msg
  2. Provjeriti sve nazive
  3. Potencijalno pretvoriti sve floatove u double da ne moramo raditi konverziju
