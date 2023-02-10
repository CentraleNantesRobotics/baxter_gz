This packages provides a custom bridge to control Baxter in Ignition/Gazebo.

The topics are the same as the real robot and use `baxter_core_msgs/JointCommand` messages.

## Command bridge

Just run `ros2 run baxter_gazebosim cmd_bridge`

## Sensor bridge

Sensors (images / range) are bridged through the classical `ros_ign_bridge`.
