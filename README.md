<img src="docs/img/thisisfine.jpeg">

# 🕊️ Mächtige Match Möve Marvin
<!-- Peak Engineering: eine Möve, die fliegt. -->

<p align="center">
  <img alt="Marvin-Loading" src="https://media.giphy.com/media/l0MYt5jPR6QX5pnqM/giphy.gif" width="260" />
  <img alt="Drone-Vibes" src="https://media.giphy.com/media/3o7aD2saalBwwftBIY/giphy.gif" width="260" />
  <img alt="Deploying-Confidence" src="https://media.giphy.com/media/xT0GqssRweIhlz209i/giphy.gif" width="260" />
</p>

<p align="center">
  <a href="https://github.com/match-MobRob2/match-drone">
    <img alt="GitHub Repo" src="https://img.shields.io/badge/GitHub-match--drone-181717?style=for-the-badge&logo=github&logoColor=white">
  </a>
  <img alt="Ubuntu" src="https://img.shields.io/badge/Ubuntu-24.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white">
  <img alt="ROS2" src="https://img.shields.io/badge/ROS%202-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white">
  <img alt="PX4" src="https://img.shields.io/badge/PX4-SITL-000000?style=for-the-badge&logo=px4&logoColor=white">
  <img alt="Gazebo" src="https://img.shields.io/badge/Gazebo-simulation-6A5ACD?style=for-the-badge">
</p>

<p align="center">
  <img alt="Last Commit" src="https://img.shields.io/github/last-commit/match-MobRob2/match-drone?style=flat-square">
  <img alt="Issues" src="https://img.shields.io/github/issues/match-MobRob2/match-drone?style=flat-square">
  <img alt="PRs" src="https://img.shields.io/github/issues-pr/match-MobRob2/match-drone?style=flat-square">
  <img alt="Repo Size" src="https://img.shields.io/github/repo-size/match-MobRob2/match-drone?style=flat-square">
  <img alt="Top Language" src="https://img.shields.io/github/languages/top/match-MobRob2/match-drone?style=flat-square">
</p>

<p align="center">
  <a href="#-overview">Overview</a> ·
  <a href="#installation-mit-setup-skript">Setup-Skript</a> ·
  <a href="#-installations-in-einzel-schritten">Install steps</a> ·
  <a href="#-running-the-demo">Demo</a> ·
  <a href="#%EF%B8%8F-custom-drone-model-setup">Custom Models</a> ·
  <a href="#-tutorial-aufgaben">Tutorials</a>
</p>

---

## 🚁 Overview

**Match Drohne** — aka. Marvin the Seagull.

<p align="center">
  <img src="docs/img/marvin.jpg" width="45%" />
  <img src="docs/img/marvin2.jpg" width="45%" />
</p>

<details>
<summary><strong>Warum Marvin? (bitte klicken, wichtig für die Wissenschaft)</strong></summary>

- Weil "Drohne" zu normal ist.
- Weil eine Möve grundsätzlich immer Recht hat.
- Weil ohne Lore kein Repo vollständig ist.

</details>

---

## Installation mit Setup Skript
<img src="docs/img/tobi.jpg" width="200">

<p>
  <img alt="Install-Hype" src="https://media.giphy.com/media/13HgwGsXF0aiGY/giphy.gif" width="260" />
  <img alt="Terminal-Wizard" src="https://media.giphy.com/media/26ufdipQqU2lhNA4g/giphy.gif" width="260" />
</p>

Installiert ROS2 (falls nicht vorhanden) und alles andere.  
**Wichtig: Nur für Ubuntu 24.04**

```bash 
cd /path/to/your/ros2_ws/src
git clone https://github.com/match-MobRob2/match-drone/ .
cd misc/
./setup.sh
````

---

## 📦 Installations in einzel Schritten

<img src="docs/img/luca.jpg" width="200">

<p>
  <img alt="Step-by-step" src="https://media.giphy.com/media/3o7aCTfyhYawdOXcFW/giphy.gif" width="240" />
  <img alt="It works on my machine" src="https://media.giphy.com/media/3o6ZtaO9BZHcOjmErm/giphy.gif" width="240" />
</p>

### 1. Navigate to the `src` Directory

```bash
cd /path/to/your/ros2_ws/src
```

### 2. Clone the Repository

```bash
git clone https://github.com/match-MobRob2/match-drone/ .
```

### 3. PX4 Setup (10–15 minutes)

<p>
  <img alt="Compiling" src="https://media.giphy.com/media/11ZSwQNWba4YF2/giphy.gif" width="240" />
</p>

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
touch COLCON_IGNORE
bash ./Tools/setup/ubuntu.sh
DONT_RUN=1 make px4_sitl gz_x500
```

### 4. Install MAVROS

```bash
cd /path/to/your/ros2_ws/
sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

### 5. Install QGroundControl

Für den Betrieb der Drohne wird QGroundControl benötigt. Damit lässt sich Telemetrie empfangen und Befehle an die Drohne senden. Auch im späteren Realbetrieb wird die Software verwendet.

<strong>Wichtig:</strong> Ohne eine verbundene QGroundControl-Instanz hebt die Drohne aus Sicherheitsgründen nicht ab.

[QGroundControl Installation Guide](docs/QGroundControl.md)

### 6. Install Custom Models

Damit eigene Drohnenmodelle genutzt werden können, müssen einige Dateien in der Simulationsumgebung angepasst bzw. ergänzt werden. Zum Glück hat ein kleiner Tastatur-Affe (ich) dafür ein Skript gemacht:

<p>
  <img alt="Keyboard Monkey" src="https://media.giphy.com/media/26tPplGWjN0xLybiU/giphy.gif" width="240" />
</p>

```bash
cd /path/to/your/ros2_ws/src/match_models/
chmod +x install_models.sh
./install_models.sh
```

Anschließend muss der Simulator erneut gebaut werden. Das dauert diesmal nicht mehr so lange:

```bash
cd /path/to/your/ros2_ws/src/PX4_Autopilot
DONT_RUN=1 make px4_sitl gz_x500
```

<strong>Wichtig:</strong> Die Modelle werden von dem "install_models.sh" skript in Ordner des Simulators kopiert. Das heißt wenn man Modelle in "match_models/sdf" oder "match_models/worlds" verändert bekommt der Simulator davon nichts mit. Nur nach erneutem ausführen des "install_models.sh" Skripts werden veränderungen übernommen.

Ein erneutes ausführen von "DONT_RUN=1 make px4_sitl gz_x500" ist bei einfachen veränderungen der SDF dateien nicht notwendig. Nur wenn man ganz neue Drohnen Varianten anlegt. Mehr details gibts hier: [Advanced Custom Modells Setup](docs/adv_drone_seup.md)

---

## 🚀 Running the Demo

Die Demo startet eins der Standardmodelle des Simulators, hebt ab, fliegt zwei meter nach vorne und landet.

<p>
  <img alt="Takeoff" src="https://media.giphy.com/media/5xaOcLDE64VMF5r6kOk/giphy.gif" width="260" />
  <img alt="Landing" src="https://media.giphy.com/media/3o7aD4kZn5k0SEvPmo/giphy.gif" width="260" />
</p>

```bash
cd /path/to/your/ros2_ws/
colcon build --symlink-install
source install/setup.bash
ros2 launch match_launch x500.launch.py
```

---

## 🛠️ Custom Drone Model Setup

For instructions on setting up the custom drone model, see [match_models/Drone_Guide.md](match_models/Drone_Guide.md).

---

## 🎓 Tutorial-Aufgaben

Eine Sammlung von Übungsmissionen mit Startcode und Musterlösungen findest du unter [docs/tutorials/README.md](docs/tutorials/README.md).

---

<details>
<summary><strong>Bonus: Marvin-Statusanzeigen (komplett seriös)</strong></summary>

* ✅ Läuft (mit genug Kaffee)
* ⚠️ Manchmal macht Marvin Marvin-Dinge
* 🧠 Debugging-Level: „kann man machen“

</details>

<img src="docs/img/grrr.png">

<!--
Easter Egg:
Du bist doof
-->

