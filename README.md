# Primitive_Planner

## Paper

Primitive-Swarm: An Ultra-lightweight and Scalable Planner for Large-scale Aerial Swarms, Accepted by T-RO. [Arxiv](https://arxiv.org/abs/2502.16887)

Author list: Jialiang Hou, Xin Zhou, Neng Pan, Ang Li, Yuxiang Guan, Chao Xu, Zhongxue Gan, Fei Gao


## Requirements

The code is tested on clean Ubuntu 20.04 with ROS noetic installation.

Install the required package toppra:
```
sudo pip3 install toppra catkin_pkg PyYAML empy matplotlib pyrfc3339
```

## Run the code

### 1. Download and compile the code

```
git clone https://github.com/ZJU-FAST-Lab/Primitive-Planner.git
cd Primitive-Planner
catkin_make -DCMAKE_BUILD_TYPE=Release
```

### 2. Generate the motion primitive library

```
cd src/scripts
python3 swarm_path.py
```
The generated motion primitive library is stored in "src/planner/plan_manage/primitive_library/".

### 3. Run the planner

In the "Primitive-Planner/" directory:
```
source devel/setup.bash # or setup.zsh
cd src/scripts
python3 gen_position_swap.py 20 # It will generate the swarm.launch with 20 drones
roslaunch primitive_planner swarm.launch
```

Wait until all the nodes have their launch process finished and keep printing "[FSM]: state: WAIT_TARGET. Waiting for trigger".

Open another terminal, publish the trigger
```
cd src/scripts
bash pub_trigger.sh
```
Then the drones (drone number is 40) will start to fly like this
<p align = "center">
<img src="misc/40_drone.gif" width = "500" height = "347" border="5" />
</p>

Change the drone number when executing "python3 gen_position_swap.py <drone_number>". 

Before starting the "roslaunch" command, please open another terminal and run "htop" to monitor the Memory usage and CPU usage. Each drone requires about 200 MB memory. Keep the htop opened for entire flight.

The computation time is printed on the terminal in blue text, named as "plan_time". 
To get the accurate computation time, please fix the CPU frequency to its maximum by
```
sudo apt install cpufrequtils
sudo cpufreq-set -g performance
```
Otherwise the CPU will run in powersave mode with low frequency.

