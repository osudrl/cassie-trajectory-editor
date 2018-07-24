
# Cassie Trajectory Tool


This tool runs on top of the the simulation program, [MuJoCo](http://www.mujoco.org/), and is designed to generate and modify walking trajectories for the bipedal robot, [Cassie](http://www.agilityrobotics.com/robots/).
The generated walking trajectories can act as the reference motion in [reinforcement learning](https://arxiv.org/abs/1803.05580) so varied walking behaviors can be learned on Cassie.


This tool was developed [Kevin Kellar](https://github.com/kkevlar) and with the mentorship of [Patrick Clary](https://github.com/pclary) for use within the [Dynamic Robotics Laboratory](http://mime.oregonstate.edu/research/drl/) at Oregon State University.


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

### Big Picture: Roadmap Diagram


![rough roadmap diagram](https://i.imgur.com/hGbAoiS.png)

The above diagram identifies many of the key source files, data structures, and fucntions of the tool.

Symbol | Description
--- | ---
Rectangle (solid outline) | Function
Rectangle (dashed outline) | Data structure
Solid Arrow | Connects function caller (base) to callee (tip)
Number on Arrow | Identifies the order that the caller calls these functions
Dashed Arrow | Data accesses and overwrites
Dashed Fence | The files (.c/.h) where these functions/structures are sourced
Diamond | Confusing decision points within functions
Circle | Distinct tasks within a method


The diagram is most useful when used in conjuction with the written description of each feature.


### Important Data Structures

Each major data structure is explained below

Sub-Heading | Information
--- | ----
Memory Location | Where in memory does this data actually live? Stackframe, globals, heap, etc.
Setup | How and where is structure is initially set up
Usages | Relevant times the structure is used in functions
Fields | List of contained fields, ordered from most to least relevant (may not be perfectly accurate in future revisions)

#### traj_info_t ([link @ v0.1](https://github.com/osudrl/cassie-trajectory-editor/blob/0dbf44c7536c35cd1c7d0dfab21b6e0a6ace8941/src/main.h#L53:L69))

The traj_info struct initally was defined just to encapsulate the [mjModel\*](http://www.mujoco.org/book/reference.html#mjModel) / [mjData\*](http://www.mujoco.org/book/reference.html#mjData) references such that method calls had the ability to modify the robot's qpos values. Now, however, the struct has expanded to hold all the current runtime information about the tool, including the pose timeline and other common structures used by MuJoCo method calls.

##### Memory Location

The traj_info struct is allocated as a global [towards the top of simulate.c](https://github.com/osudrl/cassie-trajectory-editor/blob/0dbf44c7536c35cd1c7d0dfab21b6e0a6ace8941/src/simulate.c#L135:L152), and is passed to all functions as a pointer to this definition.

##### Setup

The struct is initially set up by [simulate.c : reset_traj_info()](https://github.com/osudrl/cassie-trajectory-editor/blob/0dbf44c7536c35cd1c7d0dfab21b6e0a6ace8941/src/simulate.c#L135:L152) which is called by [simulate.c : loadmodel()](https://github.com/osudrl/cassie-trajectory-editor/blob/0dbf44c7536c35cd1c7d0dfab21b6e0a6ace8941/src/simulate.c#L534:L584).

##### Usages

Nearly ever function of the tool takes a traj_info reference because it allows access to the the timeline of poses and allows helper functions to modify the model's joint angles, external forces, etc.

##### Fields

Type / Name | Description | Initial State | Used In
--- | --- | --- | ---
[mjModel\*](http://www.mujoco.org/book/reference.html#mjModel) m | Contains the information about the simulated Cassie model | Initialized in `reset_traj_info()` with the value set by `load_model()` | When making calls to MuJoCo functions such as `mj_foward()` and `mj_step()` 
[mjData\*](http://www.mujoco.org/book/reference.html#mjData) d | Contains runtime physics data, such as joint positions, forces, and velocities | Same as above | Same as above
[mjvPerturb\*](http://www.mujoco.org/book/reference.html#mjvPerturb) pert | A reference to struct allocated in simulate.c's globals, containing data about the user dragging and dropping bodies with Ctrl+RightMouse (not anymore) / nodes | Initialized in `reset_traj_info()`: always points to the same structure allocated in globals | `main-traj.c : allow_node_transformations()` to update the dragged node's position to match the mouse's position
timeline_t timeline | A struct listing each discrete pose throughout the step duration | Initialized in [timeline.c](https://github.com/osudrl/cassie-trajectory-editor/blob/0dbf44c7536c35cd1c7d0dfab21b6e0a6ace8941/src/timeline.c#L15:L26) but will **eventually be dynamically allocated** | Most of the timeline.c functions use this field for setting / overwriting poses




