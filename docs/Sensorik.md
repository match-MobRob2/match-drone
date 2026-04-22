### Vorrausetzungen:
- Drohne fliegt langsam 
	- -> keine "schnelle" Erkennung notwendig
- Gebäude Vermessung
	- -> Urbane Gegend. 


## Sensorik:
#### Sowieso verhanden
- Internen Sensoren des Flightcontrollers (Beschleunigung, Lage)
- 3D Lidar zur Vermessung
  

#### Mögliche Zusätze:
- 1D Lidar zur höhen erkennung und zu sicheren Landung
- 2D Lidar für eine Rundherum erkennung
- Kameras zur Erkennung von Hindernissen
	- Kamera pro Arm
		- Hohes Sichtfeld
		- Moderate Auflösung (weniger Auflösung = weniger Rechenleistung)
		- Global Shutter (im gegensatz zu Rolling Shutter)
- GPS
	- Allerdings verläuft die Positionierung schon über die Roboter am Boden

  
  

### Sonstige Fünde:
- Mögliche Probleme bei der Verwendung von USB3.0 Geräten:
	- https://www.usb.org/sites/default/files/327216.pdf
	- https://mrs.fel.cvut.cz/data/papers/ICUAS22_MRS_HW.pdf
		- "RealSense cameras provide directly depth in image computed from a pair of infrared cameras. Their disadvantage is a strong GNSS signal interference"
		- **Möglicherweise auch relevant fürs WLAN**



### Uni Prag:
![[Pasted image 20250528021550.png]]
https://aerial-core.eu/wp-content/uploads/2023/10/MRS_Modular_UAV_Hardware_Platforms_for_Supporting_Research_in_Real-World_Outdoor_and_Indoor_Environments.pdf
Benutzen quasi alles mögliche (Drohnen in den Fotos):
"
Rangefinder: Garmin LiDAR lite V3, MB1340 MaxBotix ultrasound, 
• Planar LiDAR: RPLiDAR A3, 
• 3D LiDAR: Ouster OS1 and OS0 series, Velodyne VLP16, 
• Cameras: Basler Dart daA1600, Bluefox MLC200w (grayscale or RGB), 
• RGB-D cameras: Intel RealSense D435i and D455, 
• GNSS: NEO-M8N and RTK Emlid Reach M2, • Thermal camera: FLIR Lepton,
"



### Wie machen es zum Beispiel DJI Drohnen?
##### DJI Mini 4 (Low End)
- Vier Weitwinkel Kameras (160 Grad)

DJIs APAS Software: 
1. **Stereo-Matching** (≈2–3 ms): Disparitätskarten mit bis zu 30 k Tiefen­punkten/FOV.
2. **Punktewolke → Voxel-Gitter** (≈10 ms).
3. **Pfadplanung**: A*-Variante sucht Lücke innerhalb des Gittervolumens; wird alle ~40 ms neu berechnet.


### Sonstige Beispiele:
##### MIT:
https://news.mit.edu/2018/mit-csail-programming-drones-fly-face-uncertainty-0212
- Frontale Tiefenkamera






### Ganze andere Idee:
Wieso überhaupt die Drohne Objekte erkennen lassen? Da die Drohne sowieso getrackt wird von Robotern am Boden. Wieso können die Roboter am Boden nicht auch das Umfeld der Drohne beobachten und die Bodenstationen nutzt das als Objektvermeidung. Weniger Gewicht auf der Drohne, weniger Datenverkehr.