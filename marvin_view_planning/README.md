# marvin_view_planning

Portierung der Offline-View-Planning-Pipeline aus der Studienarbeit
(`vpp3d`, siehe `E:\Uni\Master\Studienarbeit\Code\vpp3d` bzw.
`student_code/260722_UAVViewPlanning/vpp3d` im `match_student_code`-Repo)
in dieses ROS-2-Workspace. **Reine Vorbereitung** -- es gibt hier noch
keine ROS-Nodes, Topics oder Launch-Files; das ist ein spaeterer Schritt.

## Was wurde uebernommen

Nur der berechnende Kern (Schritte [1]-[7]: Facetten -> Kegel-Sampling ->
Sichtbarkeitsmatrix per Raycast -> Set Cover -> Registrierungsgraph ->
optionale lokale Verfeinerung -> LoS-Tracking-Standorte -> TSP-Route). Die
komplette Visualisierungs-/CLI-Schicht des Prototyps (`viz.py`,
`stepviz.py`, `stepfigs.py`, `render_figures.py`, `inspector.py`, `run.py`,
`runlog.py`, `erosion.py` als Ablations-Experiment) wurde **bewusst nicht**
mitgenommen -- die brauchen wir hier nicht.

Neu hinzugekommen ist `pipeline.py::plan_scan_route()`: eine einzelne
Bibliotheksfunktion, die die Schritte aus `run.py` ohne Matplotlib/Open3D-
Anzeige und ohne CLI-Argumente durchfaehrt und ein `PlanResult` zurueckgibt
(Kandidatenposen, Sichtbarkeitsmatrix, gewaehlte Posen, Tracking-Standorte,
Route). Das ist der Ansatzpunkt fuer den spaeteren ROS-Node.

```python
from marvin_view_planning.pipeline import plan_scan_route

result = plan_scan_route(mesh_path="/pfad/zum/mesh.stl", method="greedy")
selected_positions = result.poses.positions[result.selected]
route_order = result.route.order          # Reihenfolge in `selected_positions`
tracking_stations = result.tracking.stations
```

Sensor-/Kinematik-Constraints stehen unveraendert in `config.toml`
(Arbeitsabstand, FoV, Einfallswinkel, Pitch-Klemmung, Tracking-Reichweite --
siehe Kommentare dort). `config.py::load_config()` sucht die Datei per
Default neben sich selbst; ein anderer Pfad kann uebergeben werden.

## Anbindung an die UI

`marvin_ui` → Tab „Mission“ → „Box ums Objekt aufziehen“: der Supervisor ruft
`python -m marvin_view_planning.plan_box <punkte.npy> <ergebnis.json> [optionen.json]`
(Karte + Live-Lidar, auf die Box zugeschnitten; Optionen = `config.toml`-Overrides aus der UI)
in einer eigenen venv auf. Das Ergebnis ist eine Vorschau (`maps/<karte>/scans/`), die im Tab
„Scan“ angezeigt und erst auf Knopfdruck zur Mission wird. venv einmalig anlegen
(Workspace-Root, Pfad = Supervisor-Parameter `vp_python`):

```bash
python3 -m venv .venv_view_planning
.venv_view_planning/bin/pip install -r src/marvin_view_planning/requirements.txt
```

## Bekannte offene Punkte

- **Python-Kompatibilitaet:** Der Original-Code (Python 3.12 im
  Studienarbeit-Repo) nutzte `tomllib` (stdlib erst ab 3.11). ROS 2 Humble
  laeuft auf Python 3.10 -- hier gefixt mit einem `tomllib`/`tomli`-Fallback
  in `config.py`. `tomli` ist auf `ubuntu` bereits installiert.
- **pip-Abhaengigkeiten kaputt/fehlend (Stand dieser Portierung, `ubuntu`-Host):**
  `numpy` 2.2.6 (pip, `~/.local`) ist inkompatibel mit dem systemweiten
  `scipy` 1.8.0 (apt, gegen NumPy 1.x gebaut) -- `import scipy` schlaegt fehl.
  Dadurch bricht auch der `open3d`-Import (zieht intern `sklearn` -> `scipy`).
  `ortools` ist gar nicht installiert. **Vor dem ersten Test/Import beheben**,
  z.B. mit einer isolierten venv fuer dieses Paket (`python3 -m venv`) statt
  System-Python zu veraendern -- ein Downgrade von System-`numpy` koennte
  andere ROS-Nodes auf diesem Host beeinflussen. Siehe `requirements.txt`.
- **Kein ROS-Interface:** Kein `msg`/`srv`, kein Node, kein Launch-File.
  `PlanResult` liefert reine NumPy-Arrays/Dataclasses; wie daraus PX4-
  Setpoints werden (analog `marvin_utils/pursuit.py`, das bereits
  `/local_planned_path` in `mavros_msgs/PositionTarget` uebersetzt), ist
  Gegenstand der eigentlichen Integration.
- Mesh-/Occluder-Auflösung, Tracking-Arbeitsraum (Zylinder-Constraints laut
  Studienarbeit-`CLAUDE.md`) etc. sind noch nicht auf das reale `marvin_drohne`-
  Setup (Sensorabmessungen, Gazebo-Welt) kalibriert.

## Tests

Ament-Standardtrio (`test_copyright.py`, `test_flake8.py`, `test_pep257.py`):

```bash
colcon test --packages-select marvin_view_planning
colcon test-result --verbose
```

`test_pipeline_smoke.py` prueft, dass die Pipeline auf der synthetischen
`box`-Szene durchlaeuft und eine nichtleere Route liefert (kein Optimalitaets-
anspruch). Braucht eine funktionierende `numpy`/`scipy`/`open3d`/`ortools`-
Umgebung (siehe oben), daher nicht Teil des Standard-Linter-Laufs:

```bash
python3 -m pytest src/marvin_view_planning/test/test_pipeline_smoke.py
```
