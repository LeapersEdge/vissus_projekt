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
- `boit-control`: aktivira se za svaki bot, *subscribe*a na svoj topic u kojem se nalaze odmetry msgs od svakog boida u svome susjedstvu, i nalazi pozu najbliže prepreke) 
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
Parametre sustava namjestite tako da publishate poruku tipa TuningParams msg. Mi preporučujemo rqt za to.
```shell
rqt
```

#### To do
  1. Testirati simulaciju i isprobati razne parametre, parametri su definirani u msg/TuningParams.msg
  2. Provjeriti sve nazive
  3. Potencijalno pretvoriti sve floatove u double da ne moramo raditi konverziju
