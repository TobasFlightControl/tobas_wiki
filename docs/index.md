# Tobas User Guide

Tobas is a tool designed to assist in the development of drones for complex missions.
It employs the Unified Robot Description Format (URDF) for intricate modeling, enhancing control capabilities
and enabling the operation of drones beyond the scope of conventional flight controllers.

## Features

---

- Utilizes URDF (Unified Robot Description Format) to model the drone's structural and mass properties.
- Incorporates the aerodynamic properties of propulsion systems and fixed-wing components.
- Addresses the nonlinear dynamics arising from the interaction between motors and propulsion systems.
- Adapts to dynamic variations resulting from alterations in joint angles.
- Features a graphical user interface (GUI) for adjusting various settings.
- Compatible with ROS (Robot Operating System), providing a consistent interface for both simulated and real-world drones.

## Capabilities of Tobas

---

### Enhanced Control and Diverse Design Possibilities

Tobas elevates control precision by factoring in elements like the drone's center of gravity and propeller arrangement.
This advanced consideration often results in superior control compared to conventional flight controllers.

Furthermore, Tobas is versatile, capable of operating any drone defined in URDF and equipped with a compatible controller.
This includes unique drone configurations such as:

- Drones with a significantly off-center gravity due to specialized sensors.
- Drones designed with non-symmetrical propeller layouts for optimal camera views.
- Drones featuring propellers that are not positioned on a uniform plane.
- Drones that are outfitted with robotic arms.

### Minimized Need for Gain Adjustment

Tobas' precise drone modeling enables the extraction and prior analysis of the translational and rotational dynamics,
independent of the drone's body.
As a result, Tobas is equipped with pre-calibrated safe gains, allowing for drone operation with minimal or no gain tuning.

### Authentic Simulation Experience

Tobas accurately models both the mass characteristics of drones and the aerodynamic aspects of their propulsion systems,
leading to highly realistic simulations.
This level of authenticity greatly reduces the necessity and expenses associated with physical testing.

The simulation can also incorporate various factors that influence flight performance, including:

- Different types of wind conditions (steady, turbulent, gusty).
- Drone modeling inaccuracies.
- Battery voltage fluctuations.
- Maximum current limits of Electronic Speed Controllers (ESCs).
- Sensor response delays and noise.

## Application Scenarios

---

### Quadcopter

An example of a common quadcopter is one that uses the DJI F450 frame kit.
This setup represents a typical quadcopter configuration and serves as a good baseline for understanding standard drone dynamics.

<iframe width="560" height="315" src="https://www.youtube.com/embed/EldjS8AnBjw?si=mdp2SFPWEta51UOP" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
<br>

### Non-planar Hexacopter

This unique hexacopter design features propellers that are angled 30 degrees from the horizontal plane.
This non-planar rotor configuration offers several advantages over traditional multirotors with planar rotor setups:

- <strong>Independent Control of Position and Attitude:</strong>
  Unlike traditional designs that require a change in drone attitude to move,
  this hexacopter can control its position and attitude independently.
  It can move sideways while keeping parallel to the ground and alter its attitude without needing to move from its position.
- <strong>Direct Horizontal Thrust:</strong>
  This design can generate horizontal thrust directly, enhancing positioning accuracy and wind resistance.
  This feature is particularly useful in challenging weather conditions or when precise movements are essential.

<iframe width="560" height="315" src="https://www.youtube.com/embed/1RIXLGmx1RA?si=ADkOlZsAMb1tHyNr" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
<br>

## System Requirements

---

### Operating System: Linux

Tobas is designed to run on Ubuntu 20.04 LTS, a popular Linux distribution.
Users are expected to have a basic understanding of Linux command line operations.

### Robotics Framework: ROS

Tobas leverages ROS (Robot Operating System) for its internal communication and APIs.
While it's possible to operate drones without in-depth ROS knowledge,
familiarity with ROS is advantageous for maximizing the capabilities of Tobas. Here are some useful resources for learning ROS:

- <a href=https://wiki.ros.org target="_blank">ROS Wiki</a>
- <a href=https://www.oreilly.com/library/view/programming-robots-with/9781449325480 target="_blank">
  Programming Robots with ROS</a>

### 3D CAD

To use Tobas, creating a URDF (Unified Robot Description Format) file based on a 3D model of the drone is necessary.
While it's possible to create a URDF without a 3D model,
having CAD skills is beneficial for accurately calculating part relationships and mass properties.
Recommended CAD tools include:

- <a href=https://www.autodesk.com/products/fusion-360 target="_blank">Fusion 360</a>
- <a href=https://www.autodesk.com/products/inventor target="_blank">Inventor</a>
- <a href=https://www.solidworks.com target="_blank">SolidWorks</a>

## Contact Details

---

For inquiries or interest in using Tobas, please reach out to:

Masayoshi Dohi<br>
E-mail: masa0u0masa1215(at)gmail.com<br>
