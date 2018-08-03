

# Introduction


The Cassie Trajectory Editor aids in designing and exporting reference walking trajectories, specifically for the [Cassie robot](http://www.agilityrobotics.com/robots/).
The exported trajectory serves as a reference in the [reinforcement learning process](https://arxiv.org/abs/1803.05580) such that the trajectory designed in this tool may be learned on a [simulated Cassie](https://github.com/osudrl/cassie-mujoco-sim) and the physical robot.


The tool *will allow* the user to either [initialize](https://github.com/osudrl/cassie-trajectory-editor/blob/docs/README.md#initialization) the timeline with a single pose or a full trajectory.
During the editing process, the tool helpes the developer visualize changes to the initial timeline.
After designing the walking gait, the user exports the finished trajectory to be used in the learning process.

<img src="https://i.imgur.com/rlcpkPP.gif" width="400"> 

# Methods and Results



## User Experience



### Keybinds



### Installation



### Calculation Delays



## Inverse Kinematics

As explained simply [here](https://medium.com/unity3danimation/overview-of-inverse-kinematics-9769a43ba956), Inverse Kinematics describes the problem of setting joint angles such that the position (dependent on these joint angles) of a body, like a foot, ends up at the desired position. 
For the trajectory tool, the IK solver performs calculations when a node is dragged and dropped while the nodes are in positional mode.


![Imgur](https://i.imgur.com/2nrSmNf.png)


If the user were to make the above transformation, the foot is in a new position for each frame in the affected segment of the timeline. 
The target positions are known, however, inverse kinematics needs to be solved to determine which joint modifications for each frame will get the foot to its target.


### Preferred Solution


The tool's master branch implements an inverse kinematics solver which relies on MuJoCo physics simulation. 
At a high level, the simulation is initialized with the robot's pose for that frame, before any solving has taken place.
Next, a [PD controller](http://robotic-controls.com/learn/programming/pd-feedback-control-introduction) applies external forces to the body, pushing it towards the target position.
When the body's position is within the accepted error range, the solution is accepted.
Once accepted, the joint positions are stored in the timeline, and the solver works on the next frame.


**INCLUDE FLOW DIAGRAM**


The diagram shows the four main elements of the current IK solver. These elements, in the order they are discussed here, are listed below:


* PD Control
* Cleanup
* Target Cutoff
* Setup

#### PD Control


If it was possible to just instantly set a body's position to the target, the IK problem would not need to be solved.
Instead of allowing direct control of end-effector positions, MuJoCo allows external forces applied to any of the bodies in the model.


The PD control stage of the solver works by dragging th body to the target position by applying these external forces.
The direction of the force vector is always toward the target position, but the magnitude is set by the output of the PD controller.
For this solver,  




#### Tuning

https://imgur.com/a/CnxWmec

https://github.com/osudrl/cassie-trajectory-editor/blob/87ed7f0df94cba1e70309e44e64a87882f006453/3dplot.py

https://github.com/osudrl/cassie-trajectory-editor/blob/87ed7f0df94cba1e70309e44e64a87882f006453/auto-liftleg.csv

https://github.com/osudrl/cassie-trajectory-editor/blob/87ed7f0df94cba1e70309e44e64a87882f006453/auto-swingleg.csv

### Dead End Solutions



#### Complete Seeding



#### Library of Poses



# Conclusion



# Contact


This tool and documentation page was written by [Kevin Kellar](https://github.com/kkevlar) for use within the [Dynamic Robotics Laboratory](http://mime.oregonstate.edu/research/drl/) at Oregon State University. 
For issues, comments, or suggestions about the tool or its documentation, feel free to [contact me](https://github.com/kkevlar) or [open a GitHub issue](https://github.com/osudrl/cassie-trajectory-editor/issues).








