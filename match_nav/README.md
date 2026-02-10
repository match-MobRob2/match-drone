# match_nav - 3D Navigation Stack

**Status**: Phase 1 MVP implementiert ✅
**Version**: 0.0.1

---

## Was wurde implementiert?

### 🎯 Phase 1 - Kernfunktionalität (MVP)

#### **Nodes** (Python)

| Node | Datei | Beschreibung |
|------|-------|--------------|
| **Global Planner** | `match_nav/global_planner.py` | Plant 3D-Pfade von Start zu Ziel. Aktuell: Einfache Geraden-Interpolation mit Waypoints. Später: OMPL RRT* mit Octomap-Kollisionsprüfung. |
| **Waypoint Follower** | `match_nav/waypoint_follower.py` | Folgt berechneten Pfaden sequentiell. State Machine mit IDLE/NAVIGATING/AVOIDING/REACHED_GOAL States. Published Setpoints an MAVROS für PX4-Kontrolle. |
| **Mission Manager** | `match_nav/mission_manager.py` | High-Level Orchestrierung. Koordiniert Global Planner und Waypoint Follower. Haupteinstiegspunkt via `/match_nav/navigate_to_pose` Service. |

#### **Custom Messages & Services**

| Typ | Datei | Zweck |
|-----|-------|-------|
| Message | `msg/NavigationStatus.msg` | Status-Updates vom Waypoint Follower (State, Distance, Waypoint-Index) |
| Service | `srv/PlanPath.srv` | Global Planning Request/Response |
| Service | `srv/NavigateToPose.srv` | High-Level Navigation Command |
| Service | `srv/CheckCollision.srv` | Kollisionsprüfung (Phase 2) |
| Service | `srv/ExploreArea.srv` | Exploration Command (Phase 3) |

⚠️ **Hinweis**: Messages/Services kompilieren nicht in ament_python. Lösung: Separates `match_nav_msgs` Package nötig (ament_cmake).

#### **Konfiguration**

| Datei | Zweck |
|-------|-------|
| `config/octomap.yaml` | Octomap Server Config (Voxel-Größe, Sensor-Modell) |
| `config/planner.yaml` | Global Planner Parameter (Bounds, Planning Time, Waypoint Spacing) |
| `config/navigation.yaml` | Waypoint Follower & Mission Manager Settings (Geschwindigkeiten, Timeouts) |
| `config/explorer.yaml` | Frontier Explorer Config (Phase 3) |

#### **Launch-Dateien**

| Datei | Beschreibung |
|-------|--------------|
| `launch/navigation.launch.py` | Startet kompletten Navigation Stack. Inkludiert `mapper.launch.py` (Gazebo + SLAM), dann Global Planner, Waypoint Follower, Mission Manager. |

#### **Dokumentation**

| Datei | Inhalt |
|-------|--------|
| `docs/NAVIGATION.md` | Vollständige Dokumentation: Installation, Quick Start, Node-Details, API-Referenz, Troubleshooting, Roadmap |

---

## Architektur (vereinfacht)

```
User/External Node
       │
       ├─► navigate_to_pose()
       │
   Mission Manager ◄─────────── status updates
       │                             │
       ├─► plan_path()               │
       │        │                    │
       ▼        ▼                    │
  Global Planner          Waypoint Follower
       │                         │
       └─► Path ────────────────►│
                                  │
                                  ├─► /mavros/setpoint_position/local
                                  │
                                  ▼
                              PX4 SITL (Drone)
```

**Workflow**:
1. User ruft `/match_nav/navigate_to_pose` → Mission Manager
2. Mission Manager fragt Global Planner: "Plane Pfad zu Ziel"
3. Global Planner gibt Waypoint-Path zurück
4. Mission Manager published Path → Waypoint Follower
5. Waypoint Follower folgt Waypoints, sendet Setpoints an MAVROS
6. PX4 fliegt Drohne zum Ziel

---

## Datenfluss

```
FAST-LIO2 SLAM
    ↓
/cloud_registered (PointCloud2)
    ↓
(Octomap Server - Phase 2)
    ↓
/octomap_binary
    ↓
Global Planner ──► /match_nav/mission_path ──► Waypoint Follower
                                                      │
                                                      ├─► /mavros/setpoint_position/local
                                                      │
                                                      └─► /match_nav/status
```

---

## Nächste Schritte

### ⚠️ Aktuelles Problem
- Custom Messages/Services kompilieren nicht (ament_python vs. ament_cmake)
- **Lösung**: Separates `match_nav_msgs` Package erstellen

### 🔧 Phase 2 - Collision Avoidance (geplant)
- Octomap Server Integration
- OMPL RRT* mit echter Kollisionsprüfung
- Collision Checker Node
- Dynamische Hindernisvermeidung

### 🚀 Phase 3 - Autonomous Exploration (geplant)
- Frontier Explorer Node
- Information Gain Evaluation
- exploration.launch.py
- Map Persistence

---

## Quick Start

```bash
# Build
cd /home/luca/match_drone
colcon build --packages-select match_nav --symlink-install
source install/setup.bash

# Launch Navigation Stack
ros2 launch match_nav navigation.launch.py

# (Nach Takeoff) Navigate zu (5, 5, 2)
ros2 service call /match_nav/navigate_to_pose match_nav/srv/NavigateToPose \
  "{goal: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 5.0, z: 2.0}, orientation: {w: 1.0}}}}"
```

---

## Dateien-Übersicht

```
match_nav/
├── match_nav/                    # Python Package
│   ├── global_planner.py         # ✅ Implementiert (MVP)
│   ├── waypoint_follower.py      # ✅ Implementiert (MVP)
│   ├── mission_manager.py        # ✅ Implementiert (MVP)
│   ├── collision_checker.py      # ⏳ Geplant (Phase 2)
│   └── frontier_explorer.py      # ⏳ Geplant (Phase 3)
├── msg/
│   └── NavigationStatus.msg      # ✅ Definiert (nicht kompiliert)
├── srv/
│   ├── PlanPath.srv              # ✅ Definiert (nicht kompiliert)
│   ├── CheckCollision.srv        # ✅ Definiert (nicht kompiliert)
│   ├── NavigateToPose.srv        # ✅ Definiert (nicht kompiliert)
│   └── ExploreArea.srv           # ✅ Definiert (nicht kompiliert)
├── config/
│   ├── octomap.yaml              # ✅ Erstellt
│   ├── planner.yaml              # ✅ Erstellt
│   ├── navigation.yaml           # ✅ Erstellt
│   └── explorer.yaml             # ✅ Erstellt
├── launch/
│   ├── navigation.launch.py      # ✅ Erstellt
│   └── exploration.launch.py     # ⏳ Geplant (Phase 3)
├── docs/
│   └── NAVIGATION.md             # ✅ Vollständige Dokumentation
├── package.xml                   # ✅ Dependencies aktualisiert
├── setup.py                      # ✅ Entry Points aktualisiert
└── README.md                     # ✅ Diese Datei
```

**Legende**:
- ✅ Implementiert und getestet
- ⏳ Geplant für nächste Phase
- ⚠️ Problem bekannt, Lösung vorhanden

---

**Erstellt**: 2026-02-10
**Team**: Match Drone (Luca, Tobi)
**Lizenz**: MIT
