# Aufgabe: Abheben, Kreis fliegen und an Punkt X landen

Die Drohne soll abheben, einen Kreis fliegen und an einem definierten Punkt X wieder landen.

## Hinweise
- Der Kreismittelpunkt kann die Startposition sein.
- Nach dem Kreisflug sollte die Drohne zum Landepunkt navigieren.

## Startbefehle
Startbefehl-Aufgabe:
```bash
ros2 launch match_launch match.launch.py mission:=exercise_takeoff_circle_land
```

Startbefehl-Musterlösung:
```bash
ros2 launch match_launch match.launch.py mission:=demo_takeoff_circle_land
```

## Ressourcen
- Startercode: [exercise_takeoff_circle_land.py](../../match_control/match_control/exercise_takeoff_circle_land.py)
- Musterlösung: [demo_takeoff_circle_land.py](../../match_control/match_control/demo_takeoff_circle_land.py)
