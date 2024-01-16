# Tobas User Guide

Tobas is a drone development support tool created to facilitate the development of drones with advanced features.
It utilizes URDF (Unified Robot Description Format) for detailed modeling,
which improves control performance and enables flying drones that traditional flight controllers cannot handle.

## Features

---

- Uses URDF for considering the structure and mass characteristics of the drone.
- Accounts for the aerodynamic characteristics of propulsion systems and fixed wings.
- Considers the nonlinear dynamics due to the coupling of motors and propulsion systems.
- Takes into account the changes in dynamics due to changes in joint angles.
- Provides a GUI for various settings.
- Compatible with ROS, offering the same interface for both simulation and actual drones.

## What Can I Do?

---

### Improved Control Performance and Expanded Design Options

Tobas offers a high degree of control due to its consideration of factors like the center of gravity and propeller placement.
This can potentially provide better control performance than traditional flight controllers.

Moreover, any drone that can be represented in URDF and has a compatible controller can be flown.
For example, Tobas supports unconventional drones such as:

- Drones with a center of gravity significantly shifted due to special sensors.
- Drones with asymmetrical propeller placements to ensure camera angles.
- Drones with propellers not aligned on the same plane.
- Drones equipped with robotic arms.

### Reduced Need for Gain Tuning

By accurately modeling the drone, it's possible to extract the dynamics of translational and rotational systems
in a body-independent form and analyze them beforehand.
Therefore, Tobas comes with pre-set safe gains, allowing users to fly drones without the need for gain adjustments.

### Realistic Simulations

The consideration of the drone's mass characteristics and the aerodynamic properties of the propulsion system
allows for realistic physical simulations.
This significantly reduces the cost of real-world testing.

Additional elements affecting flight performance can also be simulated, such as:

- Wind (steady, turbulent, gusty)
- Modeling errors of the drone
- Voltage drop in the battery
- Maximum current of ESCs
- Sensor delays and noise

## Use Cases

---

### Quadcopter

A typical quadcopter using the DJI F450 frame kit.

<iframe width="560" height="315" src="https://www.youtube.com/embed/EldjS8AnBjw?si=mdp2SFPWEta51UOP" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
<br>

### Non-planar Hexacopter

This is a hexacopter with all propellers tilted 30 degrees from the horizontal plane.
Unlike multirotors with planar rotor configurations, which need to change their attitude to move position,
non-planar rotor configuration multirotors can independently control position and attitude.
They can move laterally while maintaining a parallel orientation to the ground and change attitude while hovering.
Additionally, they can generate direct horizontal thrust, resulting in higher positioning accuracy and better wind resistance.

<iframe width="560" height="315" src="https://www.youtube.com/embed/1RIXLGmx1RA?si=ADkOlZsAMb1tHyNr" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
<br>

## Prerequisites

---

### Linux

Tobas operates on Ubuntu 20.04 LTS, a Linux distribution.
It assumes users have basic knowledge of the command line.

### ROS

Tobas uses ROS (Robot Operating System) for internal communication and APIs.
While it's possible to fly drones without ROS knowledge, understanding ROS is recommended to fully utilize Tobas' features.
Here are some resources:

- <a href=https://wiki.ros.org/en target="_blank">ROS Wiki</a>
- <a href=https://www.oreilly.com/library/view/programming-robots-with/9781449325480/ target="_blank">
  Programming Robots with ROS</a>

### 3D CAD

Tobas を使用するためには，ドローンの 3D モデルを元に URDF を作成する必要があります．
3D モデルがなくても URDF を作成することはできますが，パーツの位置関係や質量特性の計算の際に CAD が使えると便利です．
例えば以下のようなものがあります:

- <a href=https://www.autodesk.com/products/fusion-360 target="_blank">Fusion 360</a>
- <a href=https://www.autodesk.com/products/inventor target="_blank">Inventor</a>
- <a href=https://www.solidworks.com/ target="_blank">SolidWorks</a>

## Contact Information

---

If you are considering using Tobas, please feel free to contact the following:

Masayoshi Dohi<br>
E-mail: masa0u0masa1215(at)gmail.com<br>
