# 3D Modeling of the Drone

Using 3D CAD software, create a model of the drone.
There's no specific requirement for the 3D CAD software,
but it's preferable to use one that can export mesh files (_.stl or _.dae) and perform mass analysis for URDF creation.
This tutorial will use Autodesk Fusion 360.

## Points to Note When Selecting Drone Components

---

When selecting components for the drone, pay attention to the following:

### ESC

The ESC should accept PWM signals ranging from 1000us to 2000us.
BLHeli-S firmware is a proven choice.

### Propeller

For enhanced control performance, it's desirable to use propellers with known aerodynamic characteristics.
Either use a thrust measuring device such as
<a href=https://www.tytorobotics.com/pages/series-1580-1585 target="_blank">Series 1585 Thrust Stand</a>
or choose propellers with data available on
<a href=https://m-selig.ae.illinois.edu/props/propDB.html target="_blank">UIUC Propeller Data Site</a>.

### RC Receiver

Use an RC receiver compatible with S.BUS and supports at least 8 channels.
For this tutorial,
<a href=https://www.amazon.co.jp/UltraPower-Corona-R8SF-S-BUS-S-FHSS/dp/B087YZYN9W target="_blank">Corona R8SF</a>
is used.

### RC Transmitter

Make sure the RC transmitter is compatible with the protocol of the RC receiver.
In this case, <a href=https://www.rc.futaba.co.jp/products/detail/I00000006 target="_blank">Futaba T10J</a>,
which supports the S-FHSS protocol used by R8SF, as well as T-FHSS AIR for bidirectional communication, is used.

## Points to Note in Modeling

---

Keep the following in mind when modeling:

### 1. Create a Model for Each Rigid Body

In URDF, robots including drones are represented as multi-link rigid body systems.
Therefore, even at the coarsest level, it's necessary to create a model for each rigid body.
However, URDF allows the setting of fixed joints, so a single rigid body can be split into multiple parts.

### 2. Align Each Model's Axes with the Base Coordinate System of the Drone

This makes it easier to create URDF and provides clarity during runtime.
It's recommended to align with the NWU coordinate system (X-axis forward, Y-axis left, Z-axis up),
as used in Gazebo (a physical simulator).

### 3. Set Materials for Each Model to Obtain Mass Properties

If the material is unknown or the part, like a motor, is not uniform in material, consider alternative methods:

1. Calculate the average density from the actual mass of the model and set a virtual material with this density.
2. For parts with unknown materials, separate them as links and approximate their mass properties
   with mass and primitive shapes (box, sphere, cylinder) when creating the URDF.

### 4. The Model Doesn't Have to Be Exact

Control systems have robustness to modeling errors, so the mass properties of the model don't have to be exact.
While it's important to match the model and the real drone as closely as possible,
theoretically, a deviation of up to 50% in mass properties won't significantly affect stability.
For instance, lightweight components like the RC receiver or cables can be omitted from the model, as is done in this tutorial.

## Modeling a Quadcopter

---

Here is an image of the entire assembly.
<a href=https://www.amazon.co.jp/WORK-F450-Brushless-Transmitter-Accessory/dp/B09SZ7LNXB/ref=asc_df_B09SZ7LNXB/?tag=jpgo-22&linkCode=df0&hvadid=622955507000&hvpos=&hvnetw=g&hvrand=8762434071962657181&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=1009314&hvtargid=pla-1931934789257&psc=1&mcid=9454d6926b59324992d70be300c73134 target="\_blank">F450 Frame Kit</a>
is used for this model.

![F450 Assembly](resources/model_drone/assem.png)

As per point 1, the model needs to be divided into at least as many parts as there are rigid bodies.
In this case, six models are created: four propellers, a battery, and the rest.

### Modeling the Propellers

The kit includes Phantom3 9450.
A suitable model is downloaded from <a href=https://grabcad.com target="_blank">GrabCAD</a> and imported into Fusion 360.
The propeller is positioned so that its rotational axis aligns with the Z-axis, and the material is set to ABS.
Upon checking the properties, the mass is approximately 10g, which roughly matches the actual propeller.
This process is repeated for both clockwise and counter-clockwise propellers.

![Phantom 9450 CW](resources/model_drone/propeller.png)

### Modeling the Battery

<a href=https://www.amazon.co.jp/RC%E3%83%AA%E3%83%9D%E3%83%90%E3%83%83%E3%83%86%E3%83%AA%E3%83%BC30C-2250mAh-XT60%E3%83%97%E3%83%A9%E3%82%B0%E4%BB%98%E3%81%8D-%E9%A3%9B%E8%A1%8C%E6%A9%9F%E3%82%AF%E3%83%AF%E3%83%83%E3%83%89%E3%82%B3%E3%83%97%E3%82%BF%E3%83%BC%E3%83%98%E3%83%AA%E3%82%B3%E3%83%97%E3%82%BF%E3%83%BC%E3%83%89%E3%83%AD%E3%83%BC%E3%83%B3-%E3%83%AC%E3%83%BC%E3%82%B7%E3%83%B3%E3%82%B0%E3%83%9B%E3%83%93%E3%83%BC/dp/B08X1SDHZF target="_blank">SIGP 3S 2250mAh 30C</a> is used.
Although it could have been integrated into the frame model since it is fixed to the frame,
it was decided to model the battery as a separate entity to account for the possibility of future battery replacements.
Since there was no model available on GrabCAD, a simple model was created with a rectangular shape and fillets.
Aligned with the NWU coordinate system as per point 2, and since the battery material is not uniform and unknown,
it was decided to forego setting a material in CAD, as mentioned in alternative approach 2.

![SIGP 3S 2250mAh 30C](resources/model_drone/lipo.png)

### Modeling the Frame

All parts except the propellers are combined into one model for the frame.
Like the propellers, a STEP file for the frame was downloaded from GrabCAD and imported into Fusion 360.
Additional components such as the GPS mount and the Raspberry Pi 4B + Navio2
used as the flight controller were also found and imported.
The assembly is positioned to match the NWU coordinate system, and materials are set for each part.
While the arms and bolts have uniform materials, other parts like motors, ESCs,
and the battery are assigned virtual materials using alternative approach 1.
The total mass of the frame assembly is approximately 635g, closely matching the actual frame.

![F450 Frame](resources/model_drone/frame.png)

### Exporting Mesh Files and Properties

The mesh files for each model created are exported,
and notes are made of the origin positions and mass properties around the origins.
Since Fusion 360 operates on Windows and subsequent work will be done on Ubuntu,
these files are uploaded to Google Drive or a similar service to ensure they are accessible from Ubuntu.
