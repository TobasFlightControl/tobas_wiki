# Tobas Setup Assistant

Tobas Setup Assistant is a GUI tool for generating configuration files necessary to fly drones using Tobas.
It loads the URDF created on the previous page and configures items not expressed in the URDF,
such as the aerodynamics of propellers and controllers.

## Create a Catkin Workspace

---

Tobas Setup Assistant creates a ROS package containing all the necessary configuration files
for using Tobas with the user's drone.
A catkin workspace is required for this, which can be created using the following commands:

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin init
```

You can replace `catkin_ws` with any other name.

## Launch the Setup Assistant and Load the URDF

---

Launch the Tobas Setup Assistant from the terminal:

```bash
$ roslaunch tobas_setup_assistant setup_assistant.launch
```

![launch](resources/setup_assistant/launch.png)

Press the `Browse` button to select the URDF you created, and then press `Load`.
Once the URDF is loaded, the link names are displayed in a tree structure under `Frames Tree` on the top left of the screen.
Clicking on a link name highlights the corresponding link in the central model view.
All movable joint names are displayed on the top right,
and moving the bars changes the corresponding joint angles in the central model view.

![load](resources/setup_assistant/load.png)

Once loading is complete, the tabs on the left become active.
You will configure these tabs in order.

## Battery

---

Configure the battery settings according to its specifications.

![battery](resources/setup_assistant/battery.png)

## Propulsion

---

Configure the propulsion system (propellers). `Available Links` displays links that can be used as propellers.
If not displayed, check if the joint type of the propeller link in URDF Builder is `Continuous`.

![propulsion_1](resources/setup_assistant/propulsion_1.png)

Pressing the `Add` button next to a link name displays the thrust direction as an arrow in the model view
and adds a settings tab for that propeller.
If the thrust direction is incorrect, modify the `Axis` in URDF Builder.

![propulsion_2](resources/setup_assistant/propulsion_2.png)

Configure `propeller1` by entering appropriate values in `ESC Settings` and `Blade Geometry` based on the spec sheet.

![propulsion_3](resources/setup_assistant/propulsion_3.png)

In `Motor Settings`, configure the motor dynamics.
Although `Set from experimental data` is preferable,
use `Set from motor spec` if you don't have experimental data for the motor with the propeller.
Enter appropriate values based on the spec sheet.

![propulsion_4](resources/setup_assistant/propulsion_4.png)

Configure the aerodynamic characteristics of the propeller in `Aerodynamics`.
For accuracy, Set from blade geometry is not recommended. Use `Set from UIUC propeller data site` instead.
<a href=https://m-selig.ae.illinois.edu/props/propDB.html target="_blank">UIUC Propeller Data Site</a>
is a compilation of experimental aerodynamic data for various propellers.
For example, if using an APC propeller, most models' data can be found there.
However, as the data for the Phantom3 0945 propellers used here is not available, we will use the data for
<a href=https://m-selig.ae.illinois.edu/props/volume-1/data/apcsf_9x4.7_static_kt1032.txt target="_blank">APC 9 x 4.7</a>
from UIUC's Volume 1 as a substitute. Transcribe the static data into the table.

![propulsion_5](resources/setup_assistant/propulsion_5.png)

You can also create a CSV file like the one below and load it using `Load CSV`:

```csv
RPM,CT,CP
2763,0.1137,0.0481
3062,0.1152,0.0482
3310,0.1152,0.0481
3622,0.1158,0.0480
3874,0.1159,0.0480
4153,0.1163,0.0480
4422,0.1170,0.0481
4687,0.1176,0.0482
4942,0.1175,0.0481
5226,0.1184,0.0484
5473,0.1189,0.0484
5736,0.1190,0.0484
6026,0.1192,0.0484
6285,0.1192,0.0483
6554,0.1195,0.0483
6768,0.1199,0.0483
```

You must configure the other three propellers as well,
but since their settings are the same except for rotation direction, you can copy from one to the others.
Using the left tabs, copy the settings of each propeller by pressing `Copy from left tab` at the top of each tab.
Make sure the settings of `propeller1` are reflected in the other propellers
and set the `Rotating Direction` appropriately for each.
If you are unsure about the correspondence of link names and positions, use the highlight feature in the `Frames Tree` to verify.

## Fixed Wing

---

Configure the fixed-wing settings. Since this is a rotary-wing aircraft, we will skip this step.

## Custom Joints

---

Configure any joints other than the propulsion system and fixed-wing surfaces.
As there are no movable joints other than the propellers in this case, we will skip this step.

## Onboard Sensors (IMU, Barometer, GPS)

---

The 9-axis IMU, barometer, and GPS are integrated into the flight controller.
Generally, the default settings are sufficient, but since the GNSS receiver is located away from the root frame,
we will adjust the GPS offset for this case.

![gps](resources/setup_assistant/gps.png)

## Additional Sensors (Camera, LiDAR, Odometry)

---

Configure any equipment that publishes camera, LiDAR, or odometry data.
Since none of these are mounted in this case, we will skip this step.

## RC Transmitter

---

Configure the settings related to the remote control transmitter. Set `The number of flight modes` to 2.

![rc_transmitter](resources/setup_assistant/rc_transmitter.png)

## Controller

---

Configure the controller settings. Opening the combo box will display available controllers.
For this case, select `Multirotor PID`.
The `Flight Modes` should show the number of flight modes set in the `RC Transmitter` tab.
Set `Flight Mode 1` to `RollPitchYawThrust` and `Flight Mode 2` to `PosVelAccYaw`.

![controller](resources/setup_assistant/controller.png)

## Observer

---

Configure the state estimator settings.
The default settings are generally adequate.

## Simulation

---

Configure the Gazebo simulation environment settings.
`Gravity` is fixed at the standard gravitational acceleration.
The latitude, longitude, and altitude are set by default
to the Geodetic origin of Japan and the Japan Vertical Datum, respectively.
Since these are not particularly crucial for this case, we will keep the default settings.

![simulation](resources/setup_assistant/simulation.png)

## Author Info

---

Enter the name and email address of the administrator of the Tobas package generated by the Setup Assistant.

![author_info](resources/setup_assistant/author_info.png)

## ROS Package

---

Set the directory and name for generating the Tobas package.
Set the `Parent Directory` to under `src/` of your catkin workspace and enter an appropriate name in `Package Name`.
Pressing the `Generate` button will create the Tobas package in the specified directory,
and the Setup Assistant will shut down automatically.

![ros_package](resources/setup_assistant/ros_package.png)

## Build

---

Move to the catkin workspace and build the generated Tobas package:

```bash
$ cd ~/catkin_ws
$ catkin build tobas_f450_config
```
