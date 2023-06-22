# ROS Simulated Robot Data

This project demonstrates the use of ROS (Robot Operating System) to process simulated robot data, including LiDAR scans and robot poses. It generates synthetic data and visualizes it using Python and ROS.

## Dependencies

To run this project, you need to have the following dependencies installed:

- ROS (Robot Operating System): Version XYZ
- Python: Version XYZ
- numpy: Version XYZ
- matplotlib: Version XYZ

## Installation

Follow the steps below to install the necessary dependencies:

### 1. Install ROS

- [ROS Installation Instructions](http://wiki.ros.org/ROS/Installation)

### 2. Install Python

- [Python Installation Instructions](https://www.python.org/downloads/)

### 3. Install Python Packages

Open a terminal and execute the following command:

```bash
pip install numpy matplotlib
```

### 4. Clone the Repository

Clone this repository to your local machine using the following command:

```bash
git clone https://github.com/your-username/ros-simulated-robot-data.git
```

## Usage

Follow the steps below to run the project:

- Open a terminal and navigate to the project directory:

```bash
cd ros-simulated-robot-data
```

- Source the ROS environment:

```bash
source /opt/ros/kinetic/setup.bash
```

- Build the project:

```bash
catkin_make
```

- Launch the ROS nodes:

```bash
roslaunch package_name launch_file.launch
```

The visualization window will open, displaying the LiDAR scans and robot poses.
