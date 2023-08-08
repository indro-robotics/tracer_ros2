# tracer_ros2
A ROS 2 repository including all necessary scripts for a differential drive robot in a Gazebo environment. 

The simulated platform uses the Hunter SE geometry and steering characteristics and generates odometry from simulated wheel encoders and an IMU.

# Installation
To install the package, first, navigate to your colcon workspace `src` folder. Clone the repository. 
```
cd <colcon_ws>/src/
git clone https://github.com/liam-dwyer/tracer_ros2.git
```

Next, we are going to install all the dependencies needed to run the package. Navigate to the root of your colcon workspace, and run the `rosdep` commands.

**Note: must first have the rosdep package installed. If you receive an error saying that the package isn't installed, run `sudo apt install python3-rosdep2`**

```
cd <colcoN_ws>/
rosdep init #Only to be done the first time using rosdep within a workspace
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y
```

If all the dependencies were installed successfully, you should receive the following message:
```
#All required rosdeps installed successfully
```
# Running the Simulation

To run the simulation, use the launch file included in the `tracer_description` package. This includes all the files needed to visualize and launch the robot into a `Gazebo` environment. 

Run the simulation using:
```
ros2 launch tracer_description simulation.launch.py
```

This will launch the simulation environment and spawn the robot. It will also launch a joystick listener node so that the robot can be controlled using an attached controller. 

The robot can also be controlled using the teleop keyboard. The robot listens to the `/cmd_vel` topic therefore any node publishing to this topic can be used to control the robot. 
