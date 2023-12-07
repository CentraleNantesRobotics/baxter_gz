This packages provides a custom bridge to control Baxter in Ignition/Gazebo.

The topics are the same as the real robot and use `baxter_core_msgs/JointCommand` messages.

## Running the simulation

The launch files require `simple_launch`:

 - `ros2 launch baxter_gazebosim sim_launch.py` to run Gazebo
 - `ros2 launch baxter_gazebosim upload_launch.py` to spawn Baxter

    use `sliders:=True` to have joint command sliders and test the simulation (requires `slider_publisher`)

## Bridges

A custom bridge is used for joint command and end-effector range sensors (Gazebo uses 3D LiDAR, messages are converted to `sensor_msgs/Range` as for the real robot

Other topics (joint states, images) are bridged through the classical `ros_gz_bridge`.
