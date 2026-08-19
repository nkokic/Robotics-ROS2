# AMR LV3 Package

Ovaj paket sadrži čvorove za patrolu robota i detekciju objekata:

## 1. Patrolling Point Gatherer
Čvor koji prikuplja točke iz RViz-a klikanjem.

## 2. Patrolling Point Navigator
Čvor koji omogućuje robotu da prolazi kroz zadane točke redoslijedom:
- Od prve do zadnje točke (forward)
- Zatim natrag obrnutim redoslijedom (backward)
- Taj slijed se ponavlja beskonačno
- Prekida patrolu kada detektira objekt i ide prema njemu
  - Zaustavlja navigaciju kroz zadane točke
  - Kreće prema detektiranom objektu na sigurnoj udaljenosti
  - Čeka kratko vrijeme na toj lokaciji
  - Nastavlja s patrolom nakon čekanja

## 3. Object Detector
Čvor koji detektira žute objekte koristeći OAK-D kameru:
- Pretplaćuje se na sinkronizirane RGB i dubinske slike
- Detektira žuti objekt korištenjem HSV filtriranja i analize povezanih komponenti
- Projicira centroid objekta u 3D prostor
- Transformira točku iz koordinatnog sustava kamere u koordinatni sustav karte
- Objavljuje 3D poziciju objekta na temi `/detected_object_point`


## Korištenje

### Korak 1: Prikupljanje točaka

Pokrenite RViz i čvor za prikupljanje točaka:

```bash
# Terminal 1: Pokreni simulaciju (Gazebo + Nav2)
ros2 launch robot_tb4_bringup rob_manip.launch.xml

# Terminal 2: Pokreni čvor za prikupljanje točaka
source install/setup.bash
ros2 launch amr_lv3 gather_patrol_points.launch.py max_points:=5 output_file:=patrol_points.yaml
```

U RViz-u koristite "Publish Point" tool (2D Nav Goal alternative) za označavanje točaka. Kliknite na željene lokacije na mapi. Čvor će automatski spremiti točke kada dosegnete maksimalan broj.

### Korak 2: Patroliranje

Nakon što su točke prikupljene, pokrenite navigator:

```bash
# Terminal 3: Pokreni navigator
source install/setup.bash
ros2 launch amr_lv3 patrol_navigator.launch.py input_file:=patrol_points.yaml
```

Robot će automatski početi patroliranje:
1. Ići će od prve točke do zadnje
2. Zatim će se vratiti obrnutim redoslijedom (od zadnje do prve)
3. Ciklus će se ponavljati beskonačno

## Parametri

### patrolling_point_gatherer
- `max_points` (default: 5): Maksimalan broj točaka za prikupljanje
- `output_file` (default: 'patrol_points.yaml'): Naziv datoteke za spremanje točaka

### patrolling_point_navigator
- `input_file` (default: 'patrol_points.yaml'): Naziv datoteke s točkama za patroliranje
- `safe_distance` (default: 1.0): Sigurna udaljenost od detektiranog objekta u metrima
- `wait_time` (default: 3.0): Vrijeme čekanja kod objekta u sekundama



## Object Detector

### Pokretanje

```bash
# Pokreni object detector sa zadanim parametrima
ros2 launch amr_lv3 object_detector.launch.py

# Ili sa custom parametrima
ros2 launch amr_lv3 object_detector.launch.py hsv_lower:=[20,100,100] hsv_upper:=[30,255,255] target_frame:=map
```

### Parametri - object_detector

- `hsv_lower` (default: [20, 100, 100]): Donje HSV granice za detekciju žute boje [H, S, V]
- `hsv_upper` (default: [30, 255, 255]): Gornje HSV granice za detekciju žute boje [H, S, V]
- `target_frame` (default: 'map'): Ciljni koordinatni sustav za transformaciju
- `min_area` (default: 500): Minimalna površina konture da bi bila validna kao objekt

### Teme - object_detector

**Pretplate:**
- `/oakd/rgb/preview/image_raw/compressed` (sensor_msgs/CompressedImage): RGB slika s kamere
- `/oakd/rgb/preview/depth` (sensor_msgs/Image): Dubinska slika
- `/oakd/rgb/preview/camera_info` (sensor_msgs/CameraInfo): Parametri kamere
- `/tf` (tf2_msgs/TFMessage): Transformacije između koordinatnih sustava

**Objave:**
- `/detected_object_point` (geometry_msgs/Point): 3D pozicija objekta u map koordinatnom sustavu
- `/object_detector/debug_image/compressed` (sensor_msgs/CompressedImage): Debug slika s vizualizacijom detekcije


## Integracija: Patrola + Detekcija Objekata

Za potpunu funkcionalnost potrebno je pokrenuti oba čvora zajedno:

```bash
# Terminal 1: Pokreni simulaciju i navigation stack
ros2 launch robot_tb4_bringup rob_manip.launch.xml

# Terminal 2: Pokreni object detector
source install/setup.bash
ros2 launch amr_lv3 object_detector.launch.py

# Terminal 3: Pokreni patrol navigator
source install/setup.bash
ros2 launch amr_lv3 patrol_navigator.launch.py input_file:=patrol_points.yaml safe_distance:=1.0 wait_time:=3.0
```

### Tijek rada (Workflow):

1. **Robot patrolira** kroz zadane točke naprijed-natrag
2. **Object detector detektira** žuti objekt i objavljuje poziciju na `/detected_object_point`
3. **Patrol navigator prima** poruku o detektiranom objektu
4. **Robot zaustavlja** trenutnu patrolu (poziva `navigator.cancelTask()`)
5. **Robot se kreće** prema objektu, zaustavljajući se na sigurnoj udaljenosti (npr. 1m)
6. **Robot čeka** određeno vrijeme (npr. 3 sekunde) na toj lokaciji
7. **Robot nastavlja** s patrolom od točke gdje je stao

### Vizualizacija u RViz:

- Zelena linija: Planirana putanja patrole
- Plava linija: Putanja prema detektiranom objektu
- Crvena točka: Trenutna lokacija robota
- Žuti marker: Detektirani objekt (ako je postavljen)


## Napomene

- Navigator koristi `nav2_simple_commander` Basic Navigator API
- Robot automatski koristi `followWaypoints` akciju za navigaciju kroz točke
- Patroliranje je beskonačno - zaustavite node-om Ctrl+C
- Potreban je aktivan Nav2 stack prije pokretanja navigatora
- Object Detector zahtijeva pokrenuti TF tree i OAK-D kameru
- Za pregled debug slike koristite: `ros2 run rqt_image_view rqt_image_view /object_detector/debug_image/compressed`
- Ako je objekt detektiran tijekom istraživanja prethodnog objekta, nova detekcija se ignorira
- Robot će uvijek završiti trenutnu navigaciju prije nego krene na novu
