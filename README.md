# vissus_projekt

  Grupa: Gabrijel Biočić, Dino Dubinović, Quinn Lee Fletcher, Ita Poklepović, Antonio Škara.

Dependencies (links to installation guides):

- ROS2: Humble

- [ROS2 stage](https://github.com/tuw-robotics/stage_ros2/blob/humble/res/install.md) [(alternativa ako prethodni ne radi)](https://github.com/LeapersEdge/vissus_projekt/blob/main/docs/stage%20installation.md)

- [Sve iz MRS uputa](https://github.com/larics/mrs_simulation/blob/main/README.md#2-manual-installation-if-you-already-have-ros-installed)

Build:

```shell
colcon build --merge-install --symlink-install --packages-select vissus_projekt
```

### Concept

#### Nodes
Imamo *node*ove:
- `boid-control`: aktivira se za svaki bot, *subscribe*a na svoj topic u kojem se nalaze odmetry msgs od svakog boida u svome susjedstvu, i nalazi pozu najbliže prepreke) 
- `data-splitter`: pokreće se samo jednom i *publish*ati će sve topic-ove koje za svakog boida

Boid controller prima dva parametara: 
- ID od robota na kojeg se *subscribe*a (default: 0)
- način rada (default: "market")

#### Messages
- `OdometryArray`
- `TuningParams`
- `Bid`
- `Formation`
- `Task`
- `TaskList`

#### Boid control
Svaki boid koristi ROS2 topics od kojih čita/piše po sljedećim:
- *Subscribers*:
  - `/adjacency_matrix` (određuje koje se robote treba ignorirat a koje ne)
  - `/robot_<self_ID>/boid_info` (sve trenutačno dostupne i potrebne informacije o okolini)
  - `/tuning_params` (nezavisni parametri za rad i ponašanje robota)
  - `/robot_goals` (ciljevi na koji bi robot trebao doci)
  - `/formation` (podaci o formaciji jata)
- *Publishers*:
  - `/cf_<self_ID>/cmd_vel` (velocity command)

#### Data splitter
Umjesto da imamo jedan node koji upravlja svima (centralizirano upravljanje), mi **radimo decentralizirano upravljanje** (svaki boid upravlja sobom).

Pošto svaki pojedini boid ne zna sam po sebi gdje je svaki drugi boid, koristimo `data-splitter` node, gdje će se konstruirati message **svoje odometrije**, **odometrije svakog od boida kojeg vidi**, te **closest obstacle**.

`data-splitter`, dakle, uzme sve odometrije boidova i iz njih proizvede odmetry array kojeg šalje svakom pojedinačnom boidu. On je esencijalno centar informacije koji šalje svakom robotu informaciju potrebnu da napravi odluke.

![Graph node za primjer s 6 boida](https://github.com/LeapersEdge/vissus_projekt/blob/main/images/node_graph.png)

### How to use
Ovisno o tome želi li se koristiti market ili samo jednostavna verzija bez njega, koristit će se sljedeće naredbe:

Market:
```shell
cd ~/ros2_ws/src/vissus_projekt/launch
./market_start.sh
```

Jednostavna verzija:
```shell
cd ~/ros2_ws/src/vissus_projekt/launch
./simple_start.sh
```

Parametre sustave morate podesiti na dva mjesta.
Prvo u launch/launch_params.yaml morate podesiti: 
  1. Broj boida: 
  2. FOV boida
  3. Njegov vidni dosed
  4. Način rada (rendevous/market/formation)

Parametre sustava(koji su zapravo pojačanja reynolodsovih pravila) namještate tako da publishate poruku tipa TuningParams msg. Mi preporučujemo rqt za to, ali možete i lagano iskoristiti ros2 topic pub.
```shell
rqt
```

#### Parameters
It is advised to modify the parameters via rqt; change the parameters in `/tuning_params`. The parameters and what they control are listed below, along with some values found to be acceptable while testing.
- `rotation_kp`: rotation rate amplification, `1.0`
- `max_speed`: maximum speed of each boid, `1.0`
- `avoidance_range`: range at which to avoid running into other boids, `0.5`
- `avoidance_factor`: intensity with which to avoid running into other boids, `-0.3`
- `census_factor`: intensity of applied concensus rule, `0.4`
- `goal_factor`: intensity with which to move towards the goal, `0.1`

##### Rule of thumb for parameters
1. `avoidance_factor` should be sufficiently high; great enough to make sure no collisions are happening but low enough to prevent violent and sudden jerking
2. `avoidance_range` should be sufficiently small; great enough to affect boids but not too small to prevent violent and sudden jerking
3. `goal_factor` should be lower than concensus factor to allow boids to form it at a reasonable pace
