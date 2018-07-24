
# Cassie Trajectory Tool


This tool runs on top of the the simulation program, [MuJoCo](http://www.mujoco.org/), and is designed to generate and modify walking trajectories for the bipedal robot, [Cassie](http://www.agilityrobotics.com/robots/).
The generated walking trajectories can act as the reference motion in [reinforcement learning](https://arxiv.org/abs/1803.05580) so varied walking behaviors can be learned on Cassie.


This tool was developed [Kevin Kellar](https://github.com/kkevlar) and with the mentorship of [Patrick Clary](https://github.com/pclary) for use within [Dynamic Robotics Laboratory](http://mime.oregonstate.edu/research/drl/) at Oregon State University.


## Getting Started 

### Getting Started: Compilation / Dependencies

1. Clone
2. Add a MuJoCo key to the repository root directory and name it mjkey.txt
3. Install `libglfw3-dev` and `wget`
4. `make`

### Compilation: Troubleshooting

Error | Solution
--- | ---
"MuJoCo will need a product key to run the tool. Please provide a product key for MuJoCo and name it mjkey.txt." | The makefile will terminate compilation if mjkey.txt is not in the root repository directory. Please provide this file.


## Tool Source Documentation


The following sections discuss the codebase of the tool.

### Big Picture

#### Roadmap Diagram


![rough roadmap diagram](https://i.imgur.com/hGbAoiS.png)

The above diagram identifies many of the key source files, data structures, and fucntions of the tool.

Symbol | Description
--- | ---
Rectangle (solid outline) | Function
Rectangle (dashed outline) | Data structure
Solid Arrow | Connects function caller (base) to callee (tip)
Numbers on Arrows | Identifies the order that the caller calls these functions
Dashed Arrow | Data accesses and overwrites
Diamond | Confusing decision points within functions
Circle | Distinct tasks within a method


The diagram is most useful when used in conjuction with the written description of each feature.





