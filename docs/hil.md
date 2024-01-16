# Hardware in the Loop (HIL)

Hardware in the Loop (HIL) refers to conducting simulations
using actual hardware (such as a flight controller or RC transmitter) in the loop.
This allows for testing the system's behavior in real-time, thereby reducing the risks associated with real-world trials.

First, launch roscore on the Raspberry Pi:

```bash
pi@navio $ roscore
```

Next, launch Gazebo on an external PC.
Since the ROS master is on the Raspberry Pi, you need to set the environment variable accordingly.

```bash
user@pc $ export ROS_MASTER_URI=http://(ラズパイのIPアドレス):11311
user@pc $ roslaunch tobas_f450_config gazebo.launch
```

On the Raspberry Pi, launch the software necessary for HIL.

<span style="color: red;"><strong>Warning: Make sure that the propellers are removed from the motors.</strong></span>

```bash
pi@navio $ su
root@navio $ roslaunch tobas_f450_config hil.launch
```

Turning the E_STOP (CH5) on the RC transmitter on and then off will enable control of the drone in Gazebo via the RC transmitter.
<span style="color: red;"><strong>
Be aware that turning E_STOP on again will trigger an emergency stop, causing all motors to stop.
</strong></span>
On the RC transmitter, the pitch lever corresponds to the north-south (X-axis),
the roll lever to east-west (Y-axis), and the throttle lever to up-down (Z-axis) velocities.
Verify that the motor speed changes are consistent between the simulation and the real hardware.
