# Flight Test

After confirming the success of the Hardware-In-the-Loop (HIL) simulation, proceed with a test flight using the actual drone.

Turn on the mobile router and establish an SSH connection to the Raspberry Pi.

```bash
user@pc $ ssh pi@navio
```

Turn on the transmitter (remote controller) and launch the `real.launch` from the Tobas package.

```bash
pi@navio $ su
root@navio $ roslaunch tobas_f450_config real.launch
```

Similar to HIL, you can start controlling the drone by turning the E_STOP (CH5) on the transmitter on and then off.
<span style="color: red;"><strong>Be careful, turning E_STOP on again will trigger an emergency stop, causing all motors to stop.</strong></span>
The pitch lever on the transmitter corresponds to forward and backward movement,
the roll lever to left and right movement, and the throttle lever to ascending and descending speeds.

<iframe width="560" height="315" src="https://www.youtube.com/embed/EldjS8AnBjw?si=mdp2SFPWEta51UOP" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
<br>
