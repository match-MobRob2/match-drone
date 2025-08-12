# Aufgabe: Abheben, Quadrat fliegen und landen

Programmiere eine Mission, bei der die Drohne abhebt, ein Quadrat fliegt und danach wieder landet.

## Hinweise
- Jeder Schenkel des Quadrats sollte gleich lang sein.
- Nutze Offboard-Befehle, um die Wegpunkte nacheinander anzufliegen.

## Startbefehle
Startbefehl-Aufgabe:
```bash
ros2 launch match_launch match.launch.py mission:=exercice_takeoff_square_land
```

Startbefehl-Musterlösung:
```bash
ros2 launch match_launch match.launch.py mission:=demo_takeoff_square_land
```

## Ressourcen
- Startercode: [exercise_takeoff_square_land.py](../../match_control/match_control/exercise_takeoff_square_land.py)
- Musterlösung: [demo_takeoff_square_land.py](../../match_control/match_control/demo_takeoff_square_land.py)
