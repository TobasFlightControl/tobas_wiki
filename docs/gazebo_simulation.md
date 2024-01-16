# Gazebo Simulation

To simulate your drone using the Tobas package you've created, follow these steps:

## Preliminary Setup

---

First, you need to load the catkin workspace environment variables into your current shell session:

```bash
$ source ~/catkin_ws/devel/setup.bash
```

Remember, this command should be executed each time you open a new terminal session.

To automate this process and avoid manual repetition, you can append this command to your ~/.bashrc file:

```bash
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

## Launching Gazebo Simulation

---

Initiate the simulation by running the following command:

```bash
$ roslaunch tobas_f450_config gazebo.launch
```

This command will start the Gazebo simulation, placing the modeled drone at the origin.

![launch](resources/gazebo_simulation/launch.png)

## Launching Tobas Software

---

To activate the main software components, such as the controller and the state estimator, use folling command:

```bash
$ roslaunch tobas_f450_config bringup.launch
```

Wait until you see a green `[INFO] Controller is ready` message.
This indicates that the drone is prepared for flight and can be operated via the ROS API.

![bringup](resources/gazebo_simulation/bringup.png)

## Teleoperation Methods

---

### Operation via Keyboard

To control the drone using your PC's keyboard, execute:

```bash
$ roslaunch tobas_f450_config keyboard_teleop.launch
```

This will enable the drone in Gazebo to take off and hover at a certain altitude.
Follow the instructions displayed in the terminal to control the drone's position using your keyboard.

![keyboard_teleop](resources/gazebo_simulation/keyboard_teleop.png)

### Operation via GUI

Alternatively, you can control the drone using a GUI.
First, terminate the previous keyboard_teleop.launch (Ctrl + C), then run:

```bash
$ roslaunch tobas_f450_config gui_teleop.launch

```

This interface allows you to control the drone's position by adjusting the bars.
Since it's a planar multirotor, you can only control its `x`, `y`, `z`, and `yaw`.
Direct control of `roll` and `pitch` is not possible.

![gui_teleop](resources/gazebo_simulation/gui_teleop.png)

### Operation using ROS API

For application development, you can send commands to the drone using the ROS API.
This enables programmatic access to drone functionalities.
Refer to [ROS API](ros_api.md) for more details.

Start by creating a ROS package for your drone operation scripts:

```bash
$ cd ~/catkin_ws/src/
$ catkin_create_pkg my_tobas_example
```

After creating your package, proceed with the following commands to build it and load it into your shell environment:

```bash
$ catkin build my_tobas_example
$ source ~/catkin_ws/devel/setup.bash
```

Next, create a script within your ROS package.
Below is an example Python script that uses the `takeoff_action` action for takeoff
and the `command/pos_vel_acc_yaw` topic for position commands.
Place this script in the `scripts/` directory of your `my_tobas_example` package.

```python
#!/usr/bin/env python3

import rospy
import actionlib

from tobas_msgs.msg import TakeoffAction, TakeoffGoal, TakeoffResult, PosVelAccYaw

ALTITUDE = 3.0  # [m]
SIDE_LENGTH = 5.0  # [m]
INTERVAL = 5.0  # [s]


if __name__ == "__main__":
    # Initialize ROS node.
    rospy.init_node("command_square_trajectory")

    # Create a takeoff action crient.
    takeoff_client = actionlib.SimpleActionClient("takeoff_action", TakeoffAction)

    # Wait for action server.
    takeoff_client.wait_for_server()

    # Create an action goal.
    takeoff_goal = TakeoffGoal()
    takeoff_goal.target_altitude = ALTITUDE
    takeoff_goal.target_duration = INTERVAL

    # Send the action goal.
    takeoff_client.send_goal_and_wait(takeoff_goal)

    # Get the action result.
    takeoff_result: TakeoffResult = takeoff_client.get_result()
    if takeoff_result.error_code < 0:
        rospy.logerr("Takeoff action failed.")
        rospy.signal_shutdown()

    # Create a command publisher.
    command_pub = rospy.Publisher("command/pos_vel_acc_yaw", PosVelAccYaw, queue_size=1)

    # Continue to command the coordinates of the vertices of the square.
    while not rospy.is_shutdown():
        # Vertice 1
        command = PosVelAccYaw()
        command.pos.x = SIDE_LENGTH / 2
        command.pos.y = SIDE_LENGTH / 2
        command.pos.z = ALTITUDE
        command_pub.publish(command)
        rospy.sleep(INTERVAL)

        # Vertice 2
        command = PosVelAccYaw()
        command.pos.x = -SIDE_LENGTH / 2
        command.pos.y = SIDE_LENGTH / 2
        command.pos.z = ALTITUDE
        command_pub.publish(command)
        rospy.sleep(INTERVAL)

        # Vertice 3
        command = PosVelAccYaw()
        command.pos.x = -SIDE_LENGTH / 2
        command.pos.y = -SIDE_LENGTH / 2
        command.pos.z = ALTITUDE
        command_pub.publish(command)
        rospy.sleep(INTERVAL)

        # Vertice 4
        command = PosVelAccYaw()
        command.pos.x = SIDE_LENGTH / 2
        command.pos.y = -SIDE_LENGTH / 2
        command.pos.z = ALTITUDE
        command_pub.publish(command)
        rospy.sleep(INTERVAL)
```

Make sure to grant execution permission to your script:

```bash
$ chmod u+x ~/catkin_ws/src/my_tobas_example/scripts/command_square_trajectory_node.py
```

Executing this script will command the drone to take off and fly along the edges of a square.
To run the script within the drone's namespace, run:

```bash
$ rosrun my_tobas_example command_square_trajectory_node.py __ns:=f450
```

## Parameter Tuning

---

If you need to adjust parameters during flight, you can do so through a GUI interface.
Launch the parameter adjustment GUI with:

```bash
$ rosrun rqt_reconfigure rqt_reconfigure
```

![rqt_reconfigure](resources/gazebo_simulation/rqt_reconfigure.png)

This interface displays all adjustable parameters, which you can modify using sliders and editors.
Hovering your cursor over a parameter name will reveal its description.
For more detailed information, visit <a href=https://wiki.ros.org/rqt_reconfigure>rqt_reconfigure | ROS</a>.

With these steps, you should be able to fully simulate, operate,
and adjust your drone using the Tobas package in a Gazebo simulation environment.
