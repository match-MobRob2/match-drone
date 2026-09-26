# Starten

Alles aus dem Workspace-Root (`/mnt/daten/match_drone`):

```bash
colcon build --symlink-install && source install/setup.bash
```

## Bedienoberfläche (empfohlen)

Einmal starten (Sim-PC bzw. später Drohnen-PC), dann alles im Browser — auch von anderen Rechnern im Netz:

```bash
ros2 launch marvin_ui bringup.launch.py            # Sim   (sim:=false auf der Drohne)
# Browser: http://<rechner>:8088   (Sim-PC: http://192.168.188.41:8088)
# Stack-Log: Tab "Log" im Browser, oder im Terminal: tail -f <ws>/maps/.stack.log
```

- **Betrieb:** „Aufnahme starten“ (neue Karte) · „Aufnahme beenden & Karte erstellen“ (stoppt + optimiert) ·
  „Auf Karte fliegen“ · Stopp · Reset
- **Flugbereitschaft:** Ampel (MAVROS, FAST-LIO-Rate, Pose, Relokalisierung, PX4-EKF, Vision-Gate, Akku, Planer)
- **Relokalisieren:** automatisch — erst lokal um den Startplatz der Kartierung, sonst globale Suche über die
  ganze Karte (~4 s). „Startpose setzen“ (Klick wo die Drohne *jetzt* steht + Blickrichtung) ist nur noch ein
  Hinweis, falls die Halle symmetrisch ist und sie sich falsch einordnet. Bis zum Treffer ist „Relokalisierung“ rot
- **Karten** (liegen in `<ws>/maps/<name>/`, Löschen = `maps/.papierkorb/`): Ansehen, Nullpunkt setzen
  (Klick + Richtung), Ausschnitt (zwei Ecken) — Karte wird aus den Keyframes neu berechnet, Rohdaten bleiben
- **Mission** (Tab): Wegpunkt-Tabelle — x/y/z, Yaw (leer = in Flugrichtung), Wartezeit, ✋ = am Punkt auf
  „Weiter“ warten; ▲▼ umsortieren, ⌖ per Klick neu setzen. Hinzufügen per Klick (Höhe per Regler) oder
  „+ Drohnenposition“. Liegt unter `maps/<karte>/missions/`. „Mission starten“ speichert und startet (nur im Modus
  „Auf Karte fliegen“ + grüne Ampel; sonst steht darunter, warum nicht; am Boden armt sie und hebt selbst ab) ·
  Weiter · Pause · Abbrechen (hält Position) · „Jetzt landen“. Findet der Planer keinen Weg, hält die Drohne.
- **Failsafe im Flug** (nur autonom = OFFBOARD): FAST-LIO/Vision/PX4-Position/Regler weg → sofort Landung;
  Karte/Relokalisierung/Planer weg → Mission bricht ab, Drohne hält auf der Stelle (roter Kasten oben rechts:
  „Halt aufheben“ oder „Jetzt landen“). PX4-Hold (AUTO.LOITER) geht indoor nicht (braucht GPS).
- **Kamera:** Knopf unten — MJPEG-Stream der Frontkamera über `web_video_server` (Port 8089; ~15 % eines Kerns
  und ~0,5 MB/s nur solange angezeigt, sonst nichts). Topic per `camera_topic:=` am bringup.
- **Ziel:** Doppelklick in die 3D-Ansicht (Höhe per Regler)
- **3D-Ansicht:** gespeicherte Karte (grau), OctoMap des Planers (farbig nach Höhe), Drohne, Pfade, Wegpunkte

Die Befehle unten sind das, was die Oberfläche intern startet — für Tests und Fehlersuche.

Ein Launch für alles: `nav_fastlio.launch.py`. Umgeschaltet wird mit zwei Schaltern:

| | `gps:=false` (Default) | `gps:=true` |
|---|---|---|
| **`sim:=true`** (Default) | Gazebo, PX4 fliegt auf FAST-LIO (wie echt) | Gazebo, PX4 fliegt auf GPS, kein SLAM |
| **`sim:=false`** | echte Drohne, FAST-LIO + Relokalisierung | echte Drohne auf GPS (EKF2 per QGC auf GPS stellen) |

Planer, Pursuit und OctoMap (Lidar bis 25 m + Tiefenkamera bis 3 m) laufen in allen Varianten.
Sim-Welt: `world:=scale3` (Default, = `marvin_models/worlds/<world>.sdf`), Startpunkt `spawn_x/y/z:=`.
Tempo: `lookahead:=` (Default 1.0 ≈ 0,8 m/s ruhig; 1.5 ≈ 1,2 m/s, ~30 % schneller, aber doppelter Jerk und engere Abstände).

## Simulation

```bash
# Schnelltest ohne SLAM-Rechenlast (Wegpunkte, Planer)
ros2 launch marvin_launch nav_fastlio.launch.py gps:=true

# Voller Stack wie auf der echten Drohne
ros2 launch marvin_launch nav_fastlio.launch.py
```

## Echte Drohne

```bash
ros2 launch marvin_launch nav_fastlio.launch.py sim:=false
# nur Sensoren + MAVROS + LEDs + Bag-Recorder (ohne Nav-Stack):
ros2 launch marvin_launch marvin_real_base.launch.py
```

Bags landen automatisch beim Armen in `~/flight_logs/`.

## Karte aufnehmen und wiederverwenden

```bash
# 1. Mapping-Flug (map_dir muss neu sein): Halle abfliegen, am Ende nochmal
#    durch schon gesehene Bereiche (-> Loop-Closures), dann Ctrl-C
ros2 launch marvin_launch mapping.launch.py map_dir:=~/karten/halle [sim:=false]

# 2. Karte optimieren (Pose-Graph, GTSAM) -> map.pcd + map.bt
ros2 run marvin_nav map_optimizer ~/karten/halle

# 3. Flug auf der Karte (Startpose relativ zum Mapping-Start)
ros2 launch marvin_launch localization.launch.py map_dir:=~/karten/halle \
  initial_x:=0.0 initial_y:=0.0 initial_yaw:=0.0 [sim:=false]
```

Der Optimierer gibt aus, wie viele Loops er gefunden hat und wie weit er Posen
verschoben hat — 0 Loops heißt: Route hatte keine Überschneidung.

Ziel anfliegen: in RViz „2D Goal Pose“, oder

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 5.0, y: 0.0, z: 1.5}, orientation: {w: 1.0}}}"
```

## Einmalig auf der echten Drohne

- udev-Symlinks `/dev/cube_orange` (FCU) und `/dev/arduino` (LEDs), sonst `fcu_url:=` / `led_port:=` setzen
- MID360-IPs in `livox_ros_driver2/config/MID360_config.json`
- PX4 (QGC): `EKF2_EV_CTRL=11`, `EKF2_GPS_CTRL=0`, `EKF2_HGT_REF=3` (Vision; Baro läuft als Stütze mit), `EKF2_MAG_TYPE=5`,
  Hebelarm `EKF2_EV_POS_X=0.10`, `EKF2_EV_POS_Z=-0.07`
- PX4 (QGC), sanft fliegen wie im Sim-Airframe: `MPC_XY_VEL_MAX=1.5`, `MPC_TILTMAX_AIR=20`, `MC_YAWRATE_MAX=60`,
  `MPC_Z_VEL_MAX_UP=1.0`, `MPC_Z_VEL_MAX_DN=0.7` — aggressive Manöver (45°, 1 g) ließen FAST-LIO in der Höhe driften
- Lidar-Montage prüfen: Drohne gerade hinstellen, in RViz muss der Boden in `map` waagrecht liegen.
  Sonst korrigieren mit `lidar_pitch:=` (rad, Default 31°), `lidar_x:=`, `lidar_z:=`
- RealSense-Montage (`base_link → camera_link` in `marvin_real_base.launch.py`) ist aus dem Sim-Modell übernommen, nachmessen

## Drift-Test (Sim)

PX4 fliegt auf GPS, FAST-LIO + Relokalisierung laufen nur mit und werden gegen Gazebo-Ground-Truth gemessen:

```bash
ros2 launch marvin_launch nav_fastlio.launch.py ekf_gps:=true
ros2 run marvin_ui fake_gcs &     # ersetzt QGC (NAV_DLL_ACT=2 verlangt GCS-Link)
ros2 run ros_gz_bridge parameter_bridge "/world/scale3/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V" \
  --ros-args -r /world/scale3/dynamic_pose/info:=/gt_poses &
python3 src/marvin_nav/test/drift_flight.py drift.csv 5.0   # Takeoff, 2x drehen, 5-m-Quadrat, landen
python3 src/marvin_nav/test/drift_report.py drift.csv
```

Weitere Tests in `marvin_nav/test/` (Sim läuft, `fake_gcs` + GT-Bridge wie oben; `python3 -s` wegen SciPy):
- `nav_mission.py <Scale.obj> "x,y;x,y"` — Ziele über Planer, misst Abstand zu Hallenstrukturen
- `map_report.py <map_dir> <flug.csv> [Scale.obj]` — Kartengüte roh vs. optimiert gegen Ground Truth
- `lio_replay.py <bag> [param:=wert]` — FAST-LIO offline gegen eine Bag (`/rgl_lidar`, `/rgl_lidar/imu`, `/clock`), Sekunden statt Sim-Neustart

GUI auf dem lokalen Monitor aus einer SSH-Sitzung: `DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority ros2 launch ...`
