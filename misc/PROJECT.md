# Match Drone Project - Mächtige Match Möve Marvin

## Projektübersicht

**Match Drone** (Codename: "Marvin the Seagull") ist ein vollständiges **autonomes Drohnensimulations- und Entwicklungssystem** basierend auf **ROS2 Humble**, **PX4 SITL** und **Gazebo**. Das Projekt wurde für Forschung, Lehre und Entwicklung im Bereich mobile Robotik an der Hochschule entwickelt.

### Zielgruppe
- Studierende (Übungsaufgaben und Demos)
- Forscher (SLAM, Navigation, autonome Systeme)
- Entwickler (Drohnen-Kontrollsoftware)

---

## Kernfunktionalität

Das System bietet eine **vollständige Simulationsumgebung** für Drohnen mit folgenden Fähigkeiten:

### 1. Simulation
- **Gazebo Simulator** mit realistischer Physik
- **PX4 SITL** (Software-in-the-Loop) Autopilot
- Multiple Drohnenmodelle mit verschiedenen Sensorkonfigurationen
- Realistische Umgebungen (Garbsen, Innenräume, Testszenen)

### 2. Sensorik
- **Livox Mid-360 Lidar** (GPU-beschleunigt via RGL)
- **Intel RealSense** (RGB + Depth)
- **IMU** (Inertial Measurement Unit)
- **GPS** (optional, GPS-freie Varianten verfügbar)
- **Barometer, Magnetometer, Optical Flow**

### 3. SLAM & Lokalisierung
- **FAST-LIO2** (Lidar-Inertial Odometry) für Echtzeit-SLAM
- **RTAB-Map** (RGB-D SLAM) Integration
- **TF-Tree** Management für Koordinatentransformationen
- Fusion von Lidar, IMU und visuellen Daten

### 4. Kontrolle & Steuerung
- **MAVROS** Bridge (ROS2 ↔ PX4)
- **Vordefinierte Flugmissionen** (Takeoff, Landing, Square, Circle)
- **Übungsaufgaben** für Studierende mit Musterlösungen
- **Service-basierte API** für externe Steuerung

### 5. Navigation (in Entwicklung)
- Path Planning (Placeholder vorhanden)
- Collision Avoidance
- Autonomous Exploration

---

## Technologie-Stack

| Komponente | Technologie | Version |
|------------|-------------|---------|
| **OS** | Ubuntu | 24.04 LTS |
| **ROS** | ROS2 | Humble |
| **Autopilot** | PX4 | v1.15+ (SITL) |
| **Simulator** | Gazebo | Harmonic |
| **SLAM** | FAST-LIO2 | ROS2 Port |
| **Lidar** | Livox SDK2 | Custom Driver |
| **Kommunikation** | MAVROS | ROS2 Version |
| **Visualisierung** | RViz2 | ROS2 Standard |
| **Ground Control** | QGroundControl | Latest |

---

## Architektur-Überblick

```
┌─────────────────────────────────────────────────────────────────┐
│                   MATCH DRONE SYSTEM STACK                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌────────────────────────────────────────────────────────┐    │
│  │  APPLICATION LAYER                                     │    │
│  │  ├─ match_ai (AI/ML - Reserved)                        │    │
│  │  ├─ match_nav (Path Planning - In Development)         │    │
│  │  └─ match_control (Mission Control & Demos)            │    │
│  └────────────────────────────────────────────────────────┘    │
│                          ↕                                      │
│  ┌────────────────────────────────────────────────────────┐    │
│  │  PERCEPTION & MAPPING LAYER                            │    │
│  │  ├─ match_slam (FAST-LIO2 Integration)                 │    │
│  │  ├─ RTAB-Map (Visual SLAM)                             │    │
│  │  └─ DB-TSDF (Dense Mapping)                            │    │
│  └────────────────────────────────────────────────────────┘    │
│                          ↕                                      │
│  ┌────────────────────────────────────────────────────────┐    │
│  │  MIDDLEWARE & UTILITIES                                │    │
│  │  ├─ match_utils (Sensor Processing & Bridges)          │    │
│  │  ├─ MAVROS (ROS2 ↔ PX4 Bridge)                         │    │
│  │  └─ TF2 (Coordinate Transforms)                        │    │
│  └────────────────────────────────────────────────────────┘    │
│                          ↕                                      │
│  ┌────────────────────────────────────────────────────────┐    │
│  │  SIMULATION & HARDWARE ABSTRACTION                     │    │
│  │  ├─ match_models (Gazebo Models & Worlds)              │    │
│  │  ├─ RGLGazeboPlugin (GPU Lidar Simulation)             │    │
│  │  ├─ Gazebo Harmonic (Physics Simulation)               │    │
│  │  └─ PX4 SITL (Autopilot Firmware)                      │    │
│  └────────────────────────────────────────────────────────┘    │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## Datenfluss

```
Gazebo Simulator
    ↓
    ├─ RGL GPU Lidar → PointCloud2
    ├─ RealSense RGB-D → Image + Depth
    ├─ IMU → Sensor_msgs/Imu
    └─ GPS → Sensor_msgs/NavSatFix
        ↓
    match_utils (Preprocessing)
        ├─ cloud_nan_filter (NaN-Removal, Distance Filter)
        ├─ pointcloud_to_livox (Format Conversion)
        └─ imu_timemachine (Timestamp Sync)
            ↓
        SLAM Pipeline
            ├─ FAST-LIO2: Lidar + IMU → Odometry + Map
            └─ RTAB-Map: RGB-D → Loop Closure + Dense Map
                ↓
            Navigation (match_nav)
                ├─ Path Planning
                └─ Obstacle Avoidance
                    ↓
                Control (match_control)
                    ├─ High-Level Missions
                    └─ Setpoint Commands
                        ↓
                    MAVROS Bridge
                        ↓
                    PX4 Autopilot (SITL)
                        ↓
                    Gazebo Actuators (Motors)
```

---

## Modulare Struktur

Das Projekt ist in **thematische ROS2-Packages** unterteilt:

### Core Packages (`match_*`)

| Package | Zweck | Status |
|---------|-------|--------|
| **match_ai** | AI/ML Integration (Reserved) | Placeholder |
| **match_control** | Drohnen-Missions-Steuerung | Aktiv |
| **match_launch** | Launch-Orchestrierung | Aktiv |
| **match_models** | Gazebo Modelle & Welten | Aktiv |
| **match_nav** | Navigation Stack | Skeleton |
| **match_slam** | SLAM Konfiguration | Aktiv |
| **match_utils** | Utility Nodes & Bridges | Aktiv |

### Externe Dependencies

| Komponente | Repository | Zweck |
|------------|-----------|-------|
| **PX4-Autopilot** | PX4/PX4-Autopilot | Flugsteuerung (SITL) |
| **FAST_LIO_ROS2** | Fork | Lidar-Inertial SLAM |
| **RGLGazeboPlugin** | Custom | GPU Lidar Simulation |
| **DB-TSDF** | Fork | Dense Reconstruction |
| **Livox-SDK2** | Livox-SDK | Lidar SDK |
| **livox_ros_driver2** | Custom Driver | ROS2 Lidar Integration |

---

## Use Cases

### 1. Lehre (Studierende)
- **Übungsaufgaben** mit vorgegebenem Framework
- **Demo-Missionen** als Referenzimplementierungen
- **Sichere Simulationsumgebung** ohne Hardware-Risiko
- **Dokumentierte APIs** und Launch-Files

### 2. Forschung
- **SLAM-Experimente** (Lidar, Visual, Fusion)
- **Autonome Navigation** in komplexen Umgebungen
- **Sensor Fusion** (Lidar + IMU + Kamera)
- **Multi-Drohnen-Szenarien** (erweiterbar)

### 3. Entwicklung
- **Algorithmen-Prototyping** in Simulation
- **Hardware-in-the-Loop** Vorbereitung
- **Continuous Integration** mit automatisierten Tests
- **Real-World Transfer** (SITL → Hardware)

---

## Besonderheiten

### GPU-Beschleunigte Lidar-Simulation
- **RGL (RobotecGPULidar)** Plugin für Gazebo
- **Realistische Strahlverfolgung** (Ray Tracing)
- **>100Hz Scan-Raten** möglich
- **Emulation realer Livox-Sensoren**

### GPS-freie Navigation
- **Spezielle Drohnen-Variante** (`match_drohne_nogps`)
- **Nur SLAM-basierte Lokalisierung**
- **Relevant für Indoor-Flüge** und GPS-denied Umgebungen

### Modulare Drohnen-Modelle
- **5 verschiedene Konfigurationen**
- **Plug-and-Play Sensoren** (SDF-basiert)
- **Einfaches Anpassen** via `install_models.sh`

### Zeitstempel-Synchronisierung
- **`imu_timemachine`** für Sim-Time Kompatibilität
- **Löst Timing-Probleme** zwischen Gazebo und ROS2
- **Wichtig für Bag-Replay** und deterministische Tests

---

## Entwicklungs-Workflow

```bash
# 1. Modelle anpassen (optional)
cd match_models/sdf/
# ... Anpassungen ...
./install_models.sh

# 2. PX4 neu bauen (bei neuen Modellen)
cd PX4-Autopilot/
DONT_RUN=1 make px4_sitl gz_x500

# 3. ROS2 Workspace bauen
cd /path/to/ros2_ws/
colcon build --symlink-install

# 4. Source & Launch
source install/setup.bash
ros2 launch match_launch match_drohne_alles.launch.py

# 5. Separate Terminals für:
# - QGroundControl (GUI)
# - RViz2 (Visualisierung)
# - Custom Nodes (Entwicklung)
```

---

## Zukünftige Entwicklungen

### Kurzfristig (In Arbeit)
- [ ] Navigation Stack vervollständigen (`match_nav`)
- [ ] Collision Avoidance mit 3D Pointclouds
- [ ] Multi-Waypoint Missionen

### Mittelfristig (Geplant)
- [ ] AI/ML Integration (`match_ai`)
- [ ] Object Detection & Tracking
- [ ] Visual Servoing
- [ ] Schwarm-Koordination (Multi-Agent)

### Langfristig (Vision)
- [ ] Hardware-in-the-Loop (HIL) mit echter Drohne
- [ ] Outdoor-Tests mit GPS-Integration
- [ ] Autonome Exploration & Mapping
- [ ] Real-World Deployment

---

## Dokumentation & Ressourcen

### Interne Dokumentation
- [README.md](../README.md) - Installations-Guide
- [docs/tutorials/](../docs/tutorials/) - Übungsaufgaben
- [match_models/Drone_Guide.md](../match_models/Drone_Guide.md) - Custom Models
- [docs/QGroundControl.md](../docs/QGroundControl.md) - Ground Control Setup

### Externe Links
- [PX4 Documentation](https://docs.px4.io/)
- [ROS2 Humble Docs](https://docs.ros.org/en/humble/)
- [Gazebo Harmonic](https://gazebosim.org/)
- [FAST-LIO2 Paper](https://github.com/hku-mars/FAST_LIO)

---

## Team & Kontakt

**Hochschule**: Match Institut (Mobile Robotics)
**Repository**: [github.com/match-MobRob2/match-drone](https://github.com/match-MobRob2/match-drone)

### Contributors
- Luca (Lead Developer - Vibes & Chaos)
- Tobi (Setup Wizard)
- Community (Issues & PRs willkommen)

---

## Lizenz

Siehe [LICENSE](../LICENSE) für Details.

---

**Status**: Aktiv in Entwicklung (Stand: Februar 2026)
**Maintenance**: Regelmäßige Updates für ROS2 Humble + PX4 Main Branch

---

> "It's not a bug, it's a feature. Unless Marvin crashes, then it's definitely a bug."
> — Ancient Drone Proverb
