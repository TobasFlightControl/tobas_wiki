# Hardware in the Loop (HIL)

Hardware in the Loop (HIL) is a simulation technique that involves integrating actual hardware components,
such as a flight controller or an RC transmitter, into the simulation loop.
This approach enables real-time testing of the system's behavior,
significantly reducing the risks associated with conducting trials in real-world environments.

To begin, initiate `roscore` on the Raspberry Pi:

```bash
pi@navio $ roscore
```

Then, start Gazebo on a separate PC.
Remember, the ROS master is running on the Raspberry Pi, so you must set the environment variable to reflect this.

```bash
user@pc $ export ROS_MASTER_URI=http://[Raspberry Pi IP Address]:11311
user@pc $ roslaunch tobas_f450_config gazebo.launch
```

On the Raspberry Pi, launch the software needed for HIL.

<span style="color: red;"><strong>Warning: Make sure that the propellers are removed from the motors.</strong></span>

```bash
pi@navio $ su
root@navio $ roslaunch tobas_f450_config hil.launch
```

To enable drone control in Gazebo using the RC transmitter, switch the E_STOP (CH5) on the transmitter on and then off.
<span style="color: red;"><strong>
However, be cautious: activating E_STOP again will engage an emergency stop, immediately halting all motors.
</strong></span>

Regarding the controls on the RC transmitter:
the pitch lever adjusts the north-south movement (X-axis),
the roll lever controls east-west movement (Y-axis),
and the throttle lever manages the altitude (Z-axis).
Ensure that the changes in motor speed are consistent between the simulation and the actual hardware.
