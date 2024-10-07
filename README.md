
# Sensorfusion von LiDAR und Kamera

Das Repository befasst sich mit der Sensorfusion von LiDAR und Kamera durch Kalibrierung. Durch die Kombination dieser beiden Sensoren ermöglichen wir eine verbesserte Wahrnehmung und Interpretation der Umgebung von SMEC.

## Installation:
In diesem Repository werden mehrere Packages angewendet:
- [3d_camera_realsense][CAM]: Dieses Package ermöglicht die Arbeit mit der ROS-Kameranode für die Intel RealSense D435-Kamera.
- [objekterkennung][OBJ]: In diesem Package ist das trainierte Modell aus Modul 4 zur Objekterkennung enthalten.
- [YDLIDAR ROS2 PACKAGE][LIDAR]: Dieses Package stellt die Lidar-Node für die YDLIDAR-Sensoren bereit.

[CAM]: <https://git.hs-coburg.de/Autonomous_Driving/3d_camera_realsense>
[OBJ]: <https://git.hs-coburg.de/SMEC/objekterkennung>
[LIDAR]: <https://github.com/yangfuyuan/ydlidar_ros2>

## Verwendung
Um die Daten aus Kamera und LiDAR fusionieren zu können, ist es erforderlich, die beiden Sensoren zunächst zu kalibrieren.

### Lidar und Kamera Kalibrierung:
1. Datensatz aufbauen:

Die Daten aus der Kamera und dem LiDAR werden für ein Set von 4 rechteckigen Zielen, die
räumlich verteilt sind, aufgenommen. Der Fokus liegt auf den Kanten jedes Ziels.

<p align="center">
   <img src="pictures/set_calib.png?raw=true)" alt="Set up Calibration" />
</p>

2. Datenzuordnung

Im nächsten Schritt erfolgte die Datenzuordnung. Die Koordinaten im Laser-
Koordinatensystem der Punkte (Kanten der Ziele) werden in einer Datei gespeichert,
zusammen mit den entsprechenden Bildkoordinaten (Pixel)

Die Laser Punkte Koordinaten von jeder Kante werden mit RVIZ determiniert.

<p align="center">
   <img src="pictures/rviz_daten_erfassen.png?raw=true)" alt="rviz"/>
</p>

Die entsprechenden Bildkoordinaten werden mit Hilfe ein [Online Tool][TOOL] bestimmt.

[TOOL]: <https://pixspy.com/>

<p align="center">
   <img src="pictures/online_tool.png?raw=true)" alt="online_tool"/>
</p>

Mindestens 6 Punkte müssen aufgenommen werden.
Die koordinaten sollen in ein Data_file.txt Feile gespeischert werden.

<p align="center">
   <img src="pictures/Data_file.png?raw=true)" alt="Data_file" />
</p>

3. Intrinsische Parameter der Kamera

Um die Kamera intrinsische Parameter zu bestimmen, wird das ROS Image Proc Pipeline verwendet, um diese zu erhalten.

<p align="center">
   <img src="pictures/image_proc.png?raw=true)" alt="image_proc" />
</p>

Dafür soll die Kamera node gestartet werden:

```sh
ros2 launch 3d_camera_realsense realsense.launch.py
```

Das Topic /camera/color/camera_info beinhaltet die Kamera intrinsische Parameter

Die K Matrix und Distorsion Vektor d werden in einem config.yaml File gespeichert um später zu verwenden.

<p align="center">
   <img src="pictures/config_file.png?raw=true)" alt="config_file" />
</p>


4. Kalibrierung durchführen

Das Skript camera_lidar_calib.py in resource File  ausführen. Es nimmt drei Parametern als Input:
- „config_file“: Path zu config.yaml wo die intrinsische Parametern gespeichert sind.
- „data_file“: Path zum Data File wo die Koordinaten in Laser Frame und Bild fram
gespeichert sind.
- „result_file“: Path zum File wo die Ergebnisse (Die Transformation) gespeichert
werden.

Die Ergebnisse der Kalibrierung sind in Bild dargestellt:

<p align="center">
   <img src="pictures/config_output.png?raw=true)" alt="config_output"/>
</p>


5. Projektion

Um die LiDAR-Punkte auf das Kamerabild zu projizieren, muss zunächst die Kamera-Node und gestartet werden:

```sh
ros2 launch 3d_camera_realsense realsense.launch.py
```
Die LIDAR Node:
```sh
ros2 run ydlidar ydlidar_node
```
Danach soll die fusion_node aus camlidar_fusion Package ausgeführt:
```sh
ros2 run camlidar_fusion fusion_node
```
<p align="center">
   <img src="pictures/fusion.png?raw=true)" alt="Fusion Ergebniss" />
</p>

Die fusion_node publiziert das Topic /reprojection wo die Fusion angezeigt wird.

## Objekterkennung und Lidar Fusion

Wir verwenden das Modell, das im [Modul 4][MD] trainiert wurde, und führen den Detectnet Node aus:

[MD]: <https://git.hs-coburg.de/SMEC/objekterkennung>
```sh
ros2 launch ros_deep_learning detectnet.ros2.launch model_name:=ssd-mobilenet model_path:=/home/af/sd/smec/objekterkennung/model/3_durchgang/onnx/ssd-mobilenet.onnx input_blob:=input_0 output_cvg:=scores output_bbox:=boxes class_labels_path:=/home/af/sd/smec/objekterkennung/model/3_durchgang/labels.txt threshold:=0.5
```
Die fusion_node und LIDAR Nodes laufen lassen.

<p align="center">
   <img src="pictures/detect_net.png?raw=true)" alt="detectnet_bild" />
</p>

<p align="center">
   <img src="pictures/fusion_2.png?raw=true)" alt="reprojection" />
</p>

Die fusion_node veröffentlicht drei Topics für jedes detektierte Objekt:
- /obj_center von type HedgePosA: (x_m,y_m) sind die Koordinaten des nächsten Punktes, den das LIDAR erfassen kann. z_m ist die Objekt-ID-Nummer  (1-car, 2-traffic light, 3-Person, 4-traffic cone)
- /Kante_R von type HedgePosA: (x_m,y_m) ind die Koordinaten der rechten Kante des Objekts. z_m ist die Objekt-ID-Nummer.
- /Kante_L von type HedgePosA: (x_m,y_m) sind die Koordinaten der linken Kante des Objekts. z_m ist die Objekt-ID-Nummer
- /reprojection: Visualisierung der Fusion: Laser-Scans + Kamerabild + DetectNet Bounding Boxes.

<p align="center">
   <img src="pictures/output_fusion_node_termilal.png?raw=true)" alt="reprojection" />
</p>

SPECIAL THANKS - SMEC Group , This has been created by one of my Friend for our Group Project
