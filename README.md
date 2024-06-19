# Inverted Pendulum
#### Implementation of control laws on simulated inverted pendulum on ros2

![Output sample](illustrations/screen-capture.gif)

## Exercises
![Output sample](illustrations/diagram.gif)
#### 1) Single Inverted Pendulum
- **Interface** - Crete a new node which subscribes to state feedback and publishes to torque input. Give torque and check if the pendulum is behaving appropriately (You can add this node to the launch file later)
- **Balance** [Initial state - near upright] Write a controller to balance the inverted pendulum with initial state near upright position and not exactly upright
- **Swing-up** [Initial state - downward point at stable equilibrium] Write a controller to first swing up the pendulum, then balance on top.

#### 2) Double Inverted Pendulum
- Create a new pacakge with dynamics of an double inverted pendulum

## How-to
1. Clone this repository in /src folder of your ros2 workspace
2. Build and source the workspace. Navigate to your workspace directory and run
```
colcon build --symlink-install
source install/setup.bash
```
3. You can now launch the simulation
```
ros2 launch single_inverted single_inverted_pendulum.launch.py
```
4. Create a seperate 'controller' node which uses feedback to determine the torque value to accomplish tasks

5. Change the initial states by varying the values theta0 and theta_dot0 manually or by un-commenting relevant sections for random initializations
6. Custom messages definitions are used for input and feedback 



