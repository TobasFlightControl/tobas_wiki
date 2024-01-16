# ROS API

It is important to note that not all APIs are covered in the documentation or instructions provided.
The information typically focuses on the primary or most commonly used APIs.
To get a comprehensive understanding of all available APIs and their specific functions,
it is advisable to use ROS commands such as $ rostopic list during runtime.

## Topics

---

### Sensor Data

#### battery (tobas_msgs/Battery)

Battery state.

```txt
std_msgs/Header header
float64 voltage  # [V]
float64 current  # [A]
```

#### cpu (tobas_msgs/Cpu)

CPU state.

```txt
std_msgs/Header header
float64 temperature  # [celsius]
```

#### rc_input (tobas_msgs/RCInput)

RC Input received via S.BUS.

```txt
std_msgs/Header header
float64 roll    # CH1: [-1, 1]
float64 pitch   # CH2: [-1, 1]
float64 thrust  # CH3: [0, 1]
float64 yaw     # CH4: [-1, 1]
uint8 mode      # CH5: Flight Mode
bool e_stop     # CH7: Emergency Stop
bool gpsw       # CH8: General Purpose Switch
```

#### imu (sensor_msgs/Imu)

Gyro and acceleration measured by a 6-axis IMU.

```txt
std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance          # [rad^2]
geometry_msgs/Vector3 angular_velocity     # [rad/s]
float64[9] angular_velocity_covariance     # [rad^2/s^2]
geometry_msgs/Vector3 linear_acceleration  # [m/s^2]
float64[9] linear_acceleration_covariance  # [m^2/s^4]
```

#### magnetic_field (sensor_msgs/MagneticField)

3-axis geomagnetism measured by a magnetometer

```txt
std_msgs/Header header
geometry_msgs/Vector3 magnetic_field
float64[9] magnetic_field_covariance
```

#### air_pressure (sensor_msgs/FluidPressure)

Air pressure measured by a barometer

```txt
std_msgs/Header header
float64 fluid_pressure  # [Pa]
float64 variance        # [Pa^2]
```

#### gps (tobas_msgs/Gps)

GNSS position and velocity.

```txt
std_msgs/Header header

float64 latitude                 # [deg]
float64 longitude                # [deg]
float64 altitude                 # [m]
float64[9] position_covariance   # [m^2]

tobas_kdl_msgs/Vector ground_speed  # [m/s]
float64[9] velocity_covariance      # [m^2/s^2]
```

#### point_cloud (sensor_msgs/PointCloud)

Point cloud obtained from a LiDAR.

```txt
std_msgs/Header header
geometry_msgs/Point32[] points
sensor_msgs/ChannelFloat32[] channels
```

#### external_odometry (nav_msgs/Odometry)

The odometry obtained from external position estimation devices such as Visual Inertial Odometry (VIO).

```txt
std_msgs/Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
```

#### rotor_speeds (tobas_msgs/RotorSpeeds)

Rotation speeds of all rotors.

```txt
std_msgs/Header header
float64[] speeds  # [rad/s]
```

#### joint_states (sensor_msgs/JointState)

The state of custom joints.

```txt
std_msgs/Header header
string[] name
float64[] position  # [m or rad]
float64[] velocity  # [m/s or rad/s]
float64[] effort    # [N or Nm]
```

### State Estimation

#### odom (tobas_msgs/Odometry)

The position, velocity, and acceleration estimated by the state estimator, relative to the startup location.

```txt
std_msgs/Header header

tobas_msgs/Pose pose
tobas_kdl_msgs/Twist twist
tobas_kdl_msgs/Accel accel

float64[9] position_covariance              # [m^2]
float64[9] orientation_covariance           # [rad^2]
float64[9] linear_velocity_covariance       # [m^2/s^2]
float64[9] angular_velocity_covariance      # [rad^2/s^2]
float64[9] linear_acceleration_covariance   # [m^2/s^2/4]
float64[9] angular_acceleration_covariance  # [rad^2/s^4]
```

#### wind (tobas_msgs/Wind)

The estimated wind speed.

```txt
std_msgs/Header header
tobas_kdl_msgs/Vector vel  # [m/s]
```

### コマンド

Users can control the drone by publishing to these topics.

#### command/throttles (tobas_msgs/Throttles)

Throttle commanded to each ESC.

```txt
std_msgs/Header header
float64[] data  # [0, 1]
```

#### command/deflections (tobas_msgs/ControlSurfaceDeflections)

Control surface deflections for fixed-wing.

```txt
std_msgs/Header header
float64[] deflections  # [deg]
```

#### command/pos_vel_acc_yaw (tobas_msgs/PosVelAccYaw)

```txt
tobas_msgs/CommandLevel level

tobas_msgs/FrameId vel_frame
tobas_msgs/FrameId acc_frame

tobas_kdl_msgs/Vector pos  # [m]
tobas_kdl_msgs/Vector vel  # [m/s]
tobas_kdl_msgs/Vector acc  # [m/s^2]
float64 yaw                # [rad]
```

#### command/position_yaw (tobas_msgs/PositionYaw)

```txt
tobas_msgs/CommandLevel level
tobas_kdl_msgs/Vector pos  # [m]
float64 yaw                # [rad]
```

#### command/velocity_yaw (tobas_msgs/VelocityYaw)

```txt
tobas_msgs/CommandLevel level
tobas_msgs/FrameId frame_id
tobas_kdl_msgs/Vector vel  # [m/s]
float64 yaw                # [rad]

```

#### command/rpy_thrust (tobas_msgs/RollPitchYawThrust)

```txt
tobas_msgs/CommandLevel level
tobas_kdl_msgs/Euler rpy  # [rad]
float64 thrust            # [N]
```

#### command/pose_twist_accel (tobas_msgs/PoseTwistAccelCommand)

```txt
tobas_msgs/CommandLevel level

tobas_kdl_msgs/Vector pos    # Target global position [m]
tobas_kdl_msgs/Vector vel    # Target global linear velocity [m/s]
tobas_kdl_msgs/Vector acc    # Target global linear acceleration (feedforward) [m/s^2]

tobas_kdl_msgs/Euler rpy     # Target global orientation [rad]
tobas_kdl_msgs/Vector gyro   # Target local angular velocity [rad/s]
tobas_kdl_msgs/Vector dgyro  # Target local angular acceleration (feedforward) [rad/s^2]
```

#### command/speed_roll_delta_pitch (tobas_msgs/SpeedRollDeltaPitch)

```txt
float64 speed        # [m/s]
float64 roll         # [rad]
float64 delta_pitch  # [rad]
```

#### command/joint_positions (tobas_msgs/JointPositions)

Position commands for custom joints.

```txt
string[] name
float64[] data  # [m or rad]
```

#### command/joint_velocities (tobas_msgs/JointVelocities)

Velocity commands for custom joints.

```txt
string[] name
float64[] data  # [m/s or rad/s]
```

#### command/joint_efforts (tobas_msgs/JointEfforts)

Effort commands for custom joints.

```txt
string[] name
float64[] data  # [N or Nm]
```

### Gazebo

#### ground_truth/odom (tobas_msgs/Odometry)

The ground truth of position, velocity, and acceleration relative to the startup location.

```txt
std_msgs/Header header

Pose pose
tobas_kdl_msgs/Twist twist
tobas_kdl_msgs/Accel accel

float64[9] position_covariance              # [m^2]
float64[9] orientation_covariance           # [rad^2]
float64[9] linear_velocity_covariance       # [m^2/s^2]
float64[9] angular_velocity_covariance      # [rad^2/s^2]
float64[9] linear_acceleration_covariance   # [m^2/s^2/4]
float64[9] angular_acceleration_covariance  # [rad^2/s^4]
```

#### ground_truth/wind (tobas_msgs/Wind)

The ground truth of wind speed in the global coordinate system.

```txt
std_msgs/Header header
tobas_kdl_msgs/Vector vel  # [m/s]
```

## Services

---

### Gazebo

#### gazebo/charge_battery (std_srvs/Empty)

Fully charge the battery.

```txt
---
```

#### gazebo/set_wind_parameters (tobas_gazebo_plugins/SetWindParameters)

Set parameters for generating wind in the simulation.

```txt
# Request
float64 mean_speed         # [m/s] Average wind speed at 20ft above ground
float64 direction          # [rad] Wind direction (yaw angle)
float64 gust_speed_factor  # [-] Ratio of gust wind speed to steady wind speed
float64 gust_duration      # [s] Duration of the gust
float64 gust_interval      # [s] Interval between gusts

---

# Response
bool success

# 設定された値
float64 mean_speed
float64 direction
float64 gust_speed_factor
float64 gust_duration
float64 gust_interval
```

## Actions

---

#### takeoff_action (tobas_msgs/Takeoff)

```txt
# Goal
tobas_msgs/CommandLevel level
float64 target_altitude  # [m]
float64 target_duration  # [s]
float64 timeout          # [s] By default, timeout is infinite.

---

# Result
int8 error_code
int8 NO_ERROR = 0
int8 NOT_READY = -1
int8 INVALID_GOAL = -2
int8 PREEMPTED = -3
int8 TIMEOUT = -4
int8 UNKNOWN_ERROR = -5

---

# Feedback
```

#### landing_action (tobas_msgs/Land)

```txt
# Goal
tobas_msgs/CommandLevel level

---

# Result
int8 error_code
int8 NO_ERROR = 0
int8 NOT_READY = -1
int8 INVALID_GOAL = -2
int8 PREEMPTED = -3
int8 UNKNOWN_ERROR = -4

---

# Feedback
```
