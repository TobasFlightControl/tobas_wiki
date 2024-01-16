# Flight Test

Following the successful completion of the Hardware-In-the-Loop (HIL) simulation,
you can move on to a real test flight with the drone.

First, power up the mobile router and establish an SSH connection to the Raspberry Pi:

```bash
user@pc $ ssh pi@navio
```

Next, activate the RC transmitter and initiate `real.launch` from the Tobas package:

```bash
pi@navio $ su
root@navio $ roslaunch tobas_f450_config real.launch
```

Just like in the HIL simulation, you can start controlling the drone by toggling the E_STOP (CH5) on the transmitter on and then off. <span style="color: red;"><strong>Remember, reactivating E_STOP will engage an emergency stop, immediately halting all the drone's motors.</strong></span>

Regarding the controls on the RC transmitter:
the pitch lever adjusts the north-south movement (X-axis),
the roll lever controls east-west movement (Y-axis),
and the throttle lever manages the altitude (Z-axis).
Ensure that the changes in motor speed are consistent between the simulation and the actual hardware.

<iframe width="560" height="315" src="https://www.youtube.com/embed/EldjS8AnBjw?si=mdp2SFPWEta51UOP" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
<br>
