

# Introduction

<img align="right" src="https://i.imgur.com/rlcpkPP.gif" width="300"> 


The Cassie Trajectory Editor aids in designing and exporting reference walking trajectories, specifically for the [Cassie robot](http://www.agilityrobotics.com/robots/).
The exported trajectory serves as a reference in the [reinforcement learning process](https://arxiv.org/abs/1803.05580) such that the trajectory designed in this tool may be learned on a [simulated Cassie](https://github.com/osudrl/cassie-mujoco-sim) and the physical robot.


The tool *will allow* the user to either [initialize](https://github.com/osudrl/cassie-trajectory-editor/blob/docs/README.md#initialization) the timeline with a single pose or a full trajectory.
During the editing process, the tool helpes the developer visualize changes to the initial timeline.
After designing the walking gait, the user exports the finished trajectory to be used in the learning process.



# Methods and Results



## User Experience



### Keybinds



### Installation



### Calculation Delays



## Inverse Kinematics

<img align="right" src="https://i.imgur.com/2nrSmNf.png" width=250 > 

As explained simply [here](https://medium.com/unity3danimation/overview-of-inverse-kinematics-9769a43ba956), Inverse Kinematics describes the problem of setting joint angles such that the position (dependent on these joint angles) of a body, like a foot, ends up at the desired position. 
For the trajectory tool, the IK solver performs calculations when a node is dragged and dropped while the nodes are in positional mode.


If the user were to make the above transformation, the foot is in a new position for each frame in the affected segment of the timeline. 
The target positions are known, however, inverse kinematics needs to be solved to determine which joint modifications for each frame will get the foot to its target.


### Preferred Solution


The tool's master branch implements an inverse kinematics solver which relies on MuJoCo physics simulation. 
At a high level, the simulation is initialized with the robot's pose for that frame, before any solving has taken place.
Next, a [PD controller](http://robotic-controls.com/learn/programming/pd-feedback-control-introduction) applies external forces to the body, pushing it towards the target position.
When the body's position is within the accepted error range, the solution is accepted.
Once accepted, the joint positions are stored in the timeline, and the solver works on the next frame.


![flowdiagram](https://i.imgur.com/ivDmzPu.png)


The diagram shows the four main elements of the current IK solver. These elements, in the order they are discussed, are listed below:


* PD Control
* Cleanup
* Target Cutoff
* Setup

#### PD Control


If it was possible to just instantly set a body's position to the target, the IK problem would not need to be solved.
Instead of allowing direct control of end-effector positions, [MuJoCo](http://www.mujoco.org/) allows [external forces applied](http://www.mujoco.org/book/reference.html#mjcb_control) to any of the bodies in the model.


The PD control stage of the solver drags the body to the target position by [applying these external forces](https://github.com/osudrl/cassie-trajectory-editor/blob/0dbf44c7536c35cd1c7d0dfab21b6e0a6ace8941/src/pdik.c#L30).
The direction of the force vector is always toward the target position, but the magnitude is set by the output of the PD controller.
For this solver, the P term is proportional to the body's current distance to the target, while the D term is proportional to the current velocity of the body, slowing it down as it approaches the target.


Implementing a PD controller, however, requires a tuning the constants for the P and D terms.
Increasing both constants improves response time and decreases the number of simulation cycles needed to reach the target, but increasing these parameters also increases the instability of the simulation.
Though trial and error, it was determined that increasing the weighting of the derivative term was the primary factor causing simulation instability.


To determine which set of constants result in the most favorable controller, an [automated tester](https://github.com/osudrl/cassie-trajectory-editor/tree/automatedrop) was run overnight.
The [test script](https://github.com/osudrl/cassie-trajectory-editor/blob/automatedrop/auto.py) independently tested different weightings for the two terms: simulation steps (dependent variable) was measured as a function of Kp (independent) and Kd (independent).
The script generated two sets of data: [lifting the right foot up](https://github.com/osudrl/cassie-trajectory-editor/blob/87ed7f0df94cba1e70309e44e64a87882f006453/auto-liftleg.csv) and [swinging the right foot out](https://github.com/osudrl/cassie-trajectory-editor/blob/87ed7f0df94cba1e70309e44e64a87882f006453/auto-swingleg.csv).
These two datasets were each plotted in three dimensions with [this script](https://github.com/osudrl/cassie-trajectory-editor/blob/87ed7f0df94cba1e70309e44e64a87882f006453/3dplot.py).


<!---https://i.imgur.com/Hbpuxzb.png-->


Kp (Lift) | Kp (Swing)
--- | ---
![lift](https://i.imgur.com/NJNvOV6.png) | ![swing](https://i.imgur.com/ScS2J86.png)


The above set of graphs shows the 3d plot from an angle which emphasizes the Kp constant's effect on the number of simulation cycles required to solve inverse kinematics for the two different transformations.
As expected, the number of cycles decreases with a larger Kp, but the effect becomes less and less pronounced as the spring constant (Kp) gets larger and larger.


Kd (Lift) | Kd (Swing)
--- | ---
![lift](https://i.imgur.com/Ez0qpNy.png) | ![swing](https://i.imgur.com/Hbpuxzb.png)


This next set of graphs shows the 3d plot from an angle which emphasizes the Kd constant's effect


### Dead End Solutions



#### Complete Seeding



#### Library of Poses



# Conclusion



# Contact


This tool and documentation page was written by [Kevin Kellar](https://github.com/kkevlar) for use within the [Dynamic Robotics Laboratory](http://mime.oregonstate.edu/research/drl/) at Oregon State University. 
For issues, comments, or suggestions about the tool or its documentation, feel free to [contact me](https://github.com/kkevlar) or [open a GitHub issue](https://github.com/osudrl/cassie-trajectory-editor/issues).








