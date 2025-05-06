## Setup Instructions

### 1. Clone the repository

Clone this repository to your ROS workspace:

```bash
git clone https://github.com/phuc0908/car_model.git
```

### 2. Build
#### 1. Build the workspace
```bash
catkin_make
```
#### Explanation: Compiles the ROS workspace and all packages within it.

#### 2. Source the workspace environment
```bash
source devel/setup.bash
```
# Explanation: Sources the setup script to configure the environment for the workspace.

#### 3. Launch the robot model in Gazebo
```bash
roslaunch robot_model_pkg car_xacro.launch
```
