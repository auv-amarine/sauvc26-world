# SAUVC 2026 World Simulation in Gazebo Harmonic

Simulating the Singapore AUV Challenge 2026 competition environment using Gazebo Sim 8 (Harmonic).

Using a model of the [BlueROV2](https://github.com/clydemcqueen/bluerov2_gz), including support for both the base and heavy
configurations. It uses the BuoyancyPlugin,
HydrodynamicsPlugin and ThrusterPlugin.

![bluerov2_gz](images/img.png)

## Arena Spesfications

- **Pool**: 25m × 16m × 1.6m deep
- **Water simulation**: Buoyancy, underwater fog effects, and lighting
- **World**: Qualification Pool and Final Pool

## Models 

- Starting Zone
- Gates: qualification gate and final gate
- Drums: blue, red, and pinger (pinger still in development)
- Flares: orange, yellow, red, blue
- BlueROV2: bluerov2, bluerov2_heavy, bluerov2_ping (heavy and ping still in development)
- Ball (still in development)
- Golf Ball (still in development)

## Requirements

Ensure the requirements below are completed before you install the project:

* [Gazebo Sim 8 (Harmonic)](https://gazebosim.org/docs/latest/getstarted/)
* [ardupilot_gazebo for Harmonic](https://github.com/ArduPilot/ardupilot_gazebo)
* [ArduSub and MAVProxy](https://ardupilot.org/dev/docs/building-setup-linux.html)

## Setup

Install/Clone all these [requirements](#requirements). Ensure ArduSub is already built.

Clone this repository to your colcon workspace.

Add env to .bashrc or .zshrc:
   ```bash
   export GZ_SIM_RESOURCE_PATH=~/<your-colcon-workspace>/src/bluerov2_gz/models:~/ros2_ws/src/bluerov2_gz/worlds
   export GZ_SIM_SYSTEM_PLUGIN_PATH=~/ardupilot_gazebo/build
   ```

(Optional) Build ROS2 in `sauvc26_ros2/`

## Running Gazebo

Gazebo can be run with the commands below:

```bash
gz sim -v 3 -r <gazebo-world-file>
```

where `<gazebo-world-file>` should be replaced with:
* `sauvc_qualification.world` for the qualification arena
* `sauvc_final.world` for the final arena

Now launch ArduSub and ardupilot_gazebo:

```bash
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -L RATBeach -v ArduSub -f vectored --model=JSON --out=udp:0.0.0.0:14550 --console
```

Use MAVProxy to send commands to ArduSub:

```bash
arm throttle
mode alt_hold
rc 5 1550
disarm
```

## Camera

For access camera topic:

```bash
gz topic -e -t /front_camera
```

It will echo the camera’s image stream. If you want to view the camera directly in Gazebo, simply search for “Image Display” in the top bar of Gazebo.

For ROS 2 Humble applications, you need to add a bridge between Gazebo and ROS 2. Nevertheless, ros_gz_bridge does not function properly in the Harmonic version due to a known [issue](https://github.com/gazebosim/ros_gz/issues/727). Therefore, you should remove the source build of ros_gz and reinstall it using the `ros-humble-ros-gzharmonic` package.

```bash
cd ~/<your-colcon-workspace>
rm -rf src/ros_gz
rm -rf build/ros_gz*
rm -rf install/ros_gz*

sudo apt-get update
sudo apt-get install ros-humble-ros-gzharmonic

source /opt/ros/humble/setup.bash
```

Run ROS camera bridge:

```bash
ros2 run ros_gz_bridge parameter_bridge '/front_camera@sensor_msgs/msg/Image@ignition.msgs.Image'
```

Now you can view the image topic using rqt or RViz2:

```bash
ros2 run rqt_image_view rqt_image_view /front_camera
# or
rviz2
```

To change camera resolution, edit values at [bluerov2/model.sdf](./models/bluerov2/model.sdf) in line 455-456.

## MAVROS

Install MAVROS for ROS 2:

```bash
sudo apt install ros-humble-mavros ros-humble-mavros-extras
```
Launch MAVROS with the ArduSub connection:

```bash
ros2 launch mavros apm.launch fcu_url:=udp://:14550@localhost:14555
```

## ROS2 and Colcon

ROS2 users should add `ardupilot_gazebo -b ros2` and `bluerov2_gz` to the colcon workspace and use
colcon to build and manage the environment.

## Throubleshoot
* If the robot is teleporting, clean the gazebo cache and restart.
   ```bash
   ./scripts/clean_gazebo_cache.sh
   ```

## References

* https://github.com/clydemcqueen/bluerov2_gz

