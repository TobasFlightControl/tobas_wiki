# Create URDF

We will create a URDF (Unified Robot Description Format, a robot description language) based on the CAD model .
URDF is a format that describes the link structure and mass characteristics of a rigid multi-link system in XML format.
This time, we will create the following URDF:

```xml
<robot name="f450">
  <material name="material_-492512390">
    <color rgba="1 1 1 0.7" />
  </material>
  <material name="material_-496767138">
    <color rgba="0 0 1 0.7" />
  </material>
  <material name="material_-510938680">
    <color rgba="0 0 0 0.7" />
  </material>
  <link name="battery">
    <inertial>
      <mass value="0.186" />
      <origin xyz="0 0 0" rpy="0 -0 0" />
      <inertia ixx="0.000294655" ixy="0" ixz="0" iyy="0.00184636" iyz="0" izz="0.00193145" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 -0 0" />
      <geometry>
        <box size="0.106 0.035 0.026" />
      </geometry>
      <material name="material_-496767138">
        <color rgba="0 0 1 0.7" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 -0 0" />
      <geometry>
        <box size="0.106 0.035 0.026" />
      </geometry>
    </collision>
  </link>
  <link name="frame">
    <inertial>
      <mass value="0.635" />
      <origin xyz="0.001573 -0.000596 -0.003436" rpy="0 -0 0" />
      <inertia ixx="0.009613" ixy="6e-06" ixz="4.8e-05" iyy="0.009072" iyz="4e-06" izz="0.01867" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 -0 0" />
      <geometry>
        <mesh filename="file:///home/dohi/Downloads/frame.stl" scale="0.001 0.001 0.001" />
      </geometry>
      <material name="material_-510938680">
        <color rgba="0 0 0 0.7" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.008" rpy="0 -0 0" />
      <geometry>
        <box size="0.35 0.35 0.1" />
      </geometry>
    </collision>
  </link>
  <link name="propeller1">
    <inertial>
      <mass value="0.012" />
      <origin xyz="0 0 0.004" rpy="0 -0 0" />
      <inertia ixx="0.00046975" ixy="0" ixz="0" iyy="0.00046975" iyz="0" izz="9.375e-05" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 -0 0" />
      <geometry>
        <mesh filename="file:///home/dohi/Downloads/phantom3_0945_ccw.stl" scale="0.001 0.001 0.001" />
      </geometry>
      <material name="material_-492512390">
        <color rgba="1 1 1 0.7" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.004" rpy="0 -0 0" />
      <geometry>
        <cylinder radius="0.125" length="0.01" />
      </geometry>
    </collision>
  </link>
  <link name="propeller1_1">
    <inertial>
      <mass value="0.012" />
      <origin xyz="0 0 0.004" rpy="0 -0 0" />
      <inertia ixx="0.00046975" ixy="0" ixz="0" iyy="0.00046975" iyz="0" izz="9.375e-05" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 -0 0" />
      <geometry>
        <mesh filename="file:///home/dohi/Downloads/phantom3_0945_ccw.stl" scale="0.001 0.001 0.001" />
      </geometry>
      <material name="material_-492512390">
        <color rgba="1 1 1 0.7" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.004" rpy="0 -0 0" />
      <geometry>
        <cylinder radius="0.125" length="0.01" />
      </geometry>
    </collision>
  </link>
  <link name="propeller1_2">
    <inertial>
      <mass value="0.012" />
      <origin xyz="0 0 0.004" rpy="0 -0 0" />
      <inertia ixx="0.00046975" ixy="0" ixz="0" iyy="0.00046975" iyz="0" izz="9.375e-05" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 -0 0" />
      <geometry>
        <mesh filename="file:///home/dohi/Downloads/phantom3_0945_cw.stl" scale="0.001 0.001 0.001" />
      </geometry>
      <material name="material_-492512390">
        <color rgba="1 1 1 0.7" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.004" rpy="0 -0 0" />
      <geometry>
        <cylinder radius="0.125" length="0.01" />
      </geometry>
    </collision>
  </link>
  <link name="propeller1_3">
    <inertial>
      <mass value="0.012" />
      <origin xyz="0 0 0.004" rpy="0 -0 0" />
      <inertia ixx="0.00046975" ixy="0" ixz="0" iyy="0.00046975" iyz="0" izz="9.375e-05" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 -0 0" />
      <geometry>
        <mesh filename="file:///home/dohi/Downloads/phantom3_0945_cw.stl" scale="0.001 0.001 0.001" />
      </geometry>
      <material name="material_-492512390">
        <color rgba="1 1 1 0.7" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.004" rpy="0 -0 0" />
      <geometry>
        <cylinder radius="0.125" length="0.01" />
      </geometry>
    </collision>
  </link>
  <link name="root" />
  <joint name="battery_joint" type="fixed">
    <origin xyz="-0.035 0 -0.024" rpy="0 -0 0" />
    <axis xyz="0 0 0" />
    <parent link="frame" />
    <child link="battery" />
  </joint>
  <joint name="frame_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 -0 0" />
    <axis xyz="0 0 0" />
    <parent link="root" />
    <child link="frame" />
    <limit effort="0" velocity="0" lower="0" upper="0" />
  </joint>
  <joint name="propeller1_joint" type="continuous">
    <origin xyz="0.16 -0.16 0.024" rpy="0 0 -0.7854" />
    <axis xyz="0 0 1" />
    <parent link="frame" />
    <child link="propeller1" />
  </joint>
  <joint name="propeller1_joint_1" type="continuous">
    <origin xyz="-0.16 0.16 0.024" rpy="0 0 -0.7854" />
    <axis xyz="0 0 1" />
    <parent link="frame" />
    <child link="propeller1_1" />
  </joint>
  <joint name="propeller1_joint_2" type="continuous">
    <origin xyz="0.16 0.16 0.024" rpy="0 -0 0.7854" />
    <axis xyz="0 0 1" />
    <parent link="frame" />
    <child link="propeller1_2" />
  </joint>
  <joint name="propeller1_joint_3" type="continuous">
    <origin xyz="-0.16 -0.16 0.024" rpy="0 -0 0.7854" />
    <axis xyz="0 0 1" />
    <parent link="frame" />
    <child link="propeller1_3" />
  </joint>
</robot>
```

Links such as frame, battery, and propeller, and the joints connecting them are defined.

For creating a URDF, it's common to directly edit using an editor. However, this time we will use URDF Builder, a tool that allows for URDF creation via a GUI.

Launch URDF Builder from the terminal:

```bash
$ roslaunch urdf_builder urdf_builder.launch
```

![launch](resources/create_urdf/launch.png)

To create a new URDF, press the `New` button.
A link named `root` will be added to the Link tree.
Although not used this time, pressing the `Load` button allows you to load and edit a previously created URDF.

![new](resources/create_urdf/new.png)

Set an appropriate name in the `Robot Name` at the top left of the screen.
This time, we will simply use `f450`.

![robot_name](resources/create_urdf/robot_name.png)

## Setting Up the Frame Link

---

Right-click on the `Link` tree and select `Add Link` to open a dialog.
Enter `frame` for `Link Name`, `frame_joint` for `Joint Name`, select `root` for `Parent`, and press `OK`.
Then the `frame` link will be added to the tree.

![frame/add_link](resources/create_urdf/frame/add_link.png)

Configure the `frame` link.
Clicking on `frame` in the tree displays the settings screen at the bottom left.

Select the `General` tab, and you will see the link name set earlier.

![frame/general](resources/create_urdf/frame/general.png)

In the `Joint` tab, configure the joint settings.
The joint name set earlier is displayed in `Name`.
The previously set `root` is selected for `Parent`.
As `frame` is a reference link fixed to `root`, select `Fixed` for `Type` and set the `Origin` to the origin.

![frame/joint](resources/create_urdf/frame/joint.png)

In the `Visual` tab, set the visual information of the link.
Press the `Add` button to add a Visual object.
Multiple Visuals can be combined, but one is enough for now.
Since the coordinate system was aligned with the NWU coordinate system during CAD modeling,
the `Origin` can remain at the origin.
If the CAD coordinate system is misaligned, it needs adjustment here.
Select `Mesh` for `Geometry` type and choose the mesh file of the frame through the `Browse` button in `Path`.
This will visualize the model.
URDF uses the SI unit system, so the length unit is m, while Fusion360 was set to mm,
so it appears at a scale of 1000 times larger than actual size.
Each grid in the model view is 10 cm, so it appears very large.
Therefore, set the `Scale` to 0.001 to match the mesh file scale with the URDF.
In `Material`, you can set the color or texture of the Visual object.
Although it's not very important, since the propeller is white, set it arbitrarily to black ((R, G, B) = (0, 0, 0)).

![frame/visual_1](resources/create_urdf/frame/visual_1.png)
![frame/visual_2](resources/create_urdf/frame/visual_2.png)

In the `Collision` tab, set the collision detection area of the link.
Press the `Add` button to add a Collision object.
Like Visual, multiple Collisions can be combined.
Select `Box` for `Geometry` type.
It is possible to select `Mesh`,
but complex meshes might cause unstable calculations or heavy processing during collision detection,
so it's recommended to approximate with a primitive shape unless there's a special reason.
Adjust the size and position of the Box so that it just covers the Visual.
The settings are as shown in the following images.

![frame/collision_1](resources/create_urdf/frame/collision_1.png)
![frame/collision_2](resources/create_urdf/frame/collision_2.png)

In the `Inertial` tab, set the mass properties of the link.
Set the center of mass in `Origin`, the mass of the link in `Mass`,
and the elements of the inertia tensor about the center of mass in `Inertia`.
Since the coordinate system was aligned with the NWU coordinate system during CAD modeling,
you can directly transcribe the values obtained from the CAD properties.
If the CAD coordinate system is misaligned, adjustment here is necessary.
Unlike Visual, Inertial is not visualized, so it's difficult to confirm if there's a misalignment.
Also, note that all units are in the SI system (m, kg, kg\*m^2).

![frame/inertial](resources/create_urdf/frame/inertial.png)

## Setting Up the Battery Link

---

Add the battery link using `Add Link`.
Set `Link Name` to `battery`, `Joint Name` to `battery_joint`, and `Parent` to `frame`.

![battery/add_link](resources/create_urdf/battery/add_link.png)

It's good to have visual information when editing the `Joint` tab, so first open the `Visual` tab.
Press the `Add` button to add a Visual object.
Leave the `Origin` at the origin.
Since LiPo batteries are almost rectangular, select `Box` for `Geometry` type instead of using a mesh file.
Input the actual size of the battery in `Length`, `Width`, `Height`.
Set `Material` to blue ((R, G, B) = (0, 0, 1)), as the actual battery is predominantly blue.

![battery/visual_1](resources/create_urdf/battery/visual_1.png)
![battery/visual_2](resources/create_urdf/battery/visual_2.png)

In the `Joint` tab, configure the joint settings.
As the `battery` is fixed to the `frame`, select `Fixed` for `Type`.
Adjust the `Origin` while looking at the model view.

![battery/joint_1](resources/create_urdf/battery/joint_1.png)
![battery/joint_2](resources/create_urdf/battery/joint_2.png)

In the `Collision` tab, set the collision detection area of the link.
Press the `Add` button to add a Collision object.
Set the `Origin` and `Geometry` exactly the same as in `Visual`.

![battery/collision](resources/create_urdf/battery/collision.png)

In the `Inertial` tab, set the mass properties of the link.
Approximate the mass properties of the battery with a rectangular prism.
The center of mass then coincides with the joint origin, so set all elements of `Origin` to 0.
Input the mass of the battery in `Mass`.
Clicking on `Box Inertia` in `Inertia` opens a dialog where you input `Length`, `Width`, `Height` from earlier,
and the inertia tensor of the rectangular prism is reflected in the tab.
It's noticeable that `iyy` and `izz` are larger than `ixx`, which seems consistent.

![battery/inertial_1](resources/create_urdf/battery/inertial_1.png)
![battery/inertial_2](resources/create_urdf/battery/inertial_2.png)

In this way, links can be defined using only primitive shapes.
Describing links parametrically allows for easy modifications and should be actively utilized,
especially during the prototyping phase.

## Setting Up the Propeller Link

---

Add a propeller link from `Add Link`.
Set `Link Name` to `propeller1`, `Joint Name` to `propeller1_joint`, and `Parent` to `frame`.

![propeller/add_link](resources/create_urdf/propeller/add_link.png)

Open the `Visual` tab and add a Visual object with the `Add` button.
Leave `Origin` as the default.
Select `Mesh` as the `Type` under `Geometry` and set the `Path`.
Adjust the `Scale` if necessary.
Set the `Material` to white ((R, G, B) = (1, 1, 1)), matching the real object.

![propeller/visual](resources/create_urdf/propeller/visual.png)

Configure the joint in the `Joint` tab.
Since the propeller rotates infinitely relative to the frame, select `Continuous` for `Type`.
Adjust the `Origin` while looking at the model view.
Since the propeller rotates around the Z-axis, set the `Axis` to (X, Y, Z) = (0, 0, 1).

![propeller/joint_1](resources/create_urdf/propeller/joint_1.png)
![propeller/joint_2](resources/create_urdf/propeller/joint_2.png)

Set the contact detection area of the link in the `Collision` tab. Pressing the `Add` button adds a Collision object. Select `Cylinder` as the `Type` under `Geometry`, and adjust the `Origin`, `Radius`, and `Length` of the `Cylinder` to just cover the Visual object.

![propeller/collision_1](resources/create_urdf/propeller/collision_1.png)
![propeller/collision_2](resources/create_urdf/propeller/collision_2.png)

Set the mass properties of the link in the `Inertial` tab. Approximate it with a cylinder, similar to `Collision`. Set the `Origin` to the same value as `Collision`, and enter the mass of the propeller in `Mass`. Clicking on `Cylinder Inertia` under `Inertia` opens a dialog. Entering the `Radius` and `Length` of `Collision` reflects the inertia tensor of the cylinder in the tab. It is apparent that `ixx` and `iyy` are larger compared to `izz`, which seems consistent.

![propeller/inertial_1](resources/create_urdf/propeller/inertial_1.png)
![propeller/inertial_2](resources/create_urdf/propeller/inertial_2.png)

Since there are four propellers in total, the settings for the other three are mostly the same,
so cloning `propeller1` is faster.
Right-click on the `Link` tree with `propeller1` selected and choose `Clone Link`.
This creates `propeller1_1`, which is identical to the original except for the link and joint names.
In this case, the only differences between each propeller are the signs of the joint origin and the mesh file in `Visual`. Modify the sign of `Origin` in the `Joint` tab of `propeller1_1` and adjust the `Path` in the `Geometry` of the `Visual` tab. Follow the same procedure to add two more propellers, resulting in a total of four propellers set up.

![propeller/clone](resources/create_urdf/propeller/clone.png)

## URDF の保存

---

When you press the `Save` or `Save As` button, a dialog will appear.
Set an appropriate name and press the `Save` button to save the URDF.
Once saved, you can close the URDF Builder.

![save](resources/create_urdf/save.png)
