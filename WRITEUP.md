

# Introduction

<img align="right" src="https://i.imgur.com/rlcpkPP.gif" width="300"> 

The Cassie Trajectory Editor aids in designing reference trajectories for the [Cassie robot](http://www.agilityrobotics.com/robots/). 
The trajectories serve as targets in the [reinforcement learning process](https://arxiv.org/abs/1803.05580). 
Both physical and [simulated Cassie](https://github.com/osudrl/cassie-mujoco-sim) can learn different walking gaits through this process.

The tool allows  the user to either [initialize the timeline](https://github.com/osudrl/cassie-trajectory-editor/blob/docs/README.md#initialization) ([#5](https://github.com/osudrl/cassie-trajectory-editor/issues/5)) with a single pose or a full trajectory. 
While editing, the tool helps the user visualize changes to the initial timeline. 
After designing the gait, the user exports the reference trajectory to seed the learning process.

# Methods and Results



## Modification Tools

https://imgur.com/a/EUN9YRz

https://imgur.com/a/EUN9YRz

### Gaussian Smoothing



### Target-Based Transformation


## Inverse Kinematics

<img align="right" src="https://i.imgur.com/2nrSmNf.png" width=250 > 

[Inverse kinematics](https://medium.com/unity3danimation/overview-of-inverse-kinematics-9769a43ba956) is finding a set of joint angles such that the position of a body ends up at a target.
If the user drags the robot's foot to a new position, we need to know what hip, knee and tarsus joints will get the foot there.
Every time the user drags a set of nodes (with some exceptions), the program solves inverse kinematics.

If the user were to make the above transformation, the foot is in a new position for each frame for most of the timeline.
The solver knows the target position for each discreet pose.
But only inverse kinematics can determine what joint angles, will get the foot to its target.


### Preferred Solution


The tool implements an inverse kinematics solver which relies on MuJoCo physics simulation.
At a high level, the simulation first initializes the robot's pose.
Next, a [PD controller](http://robotic-controls.com/learn/programming/pd-feedback-control-introduction) applies external forces to the body, pushing it towards the target position.
The solver accepts the current pose as a solution when the body's position is within the accepted error range.
Once accepted, we store the joint positions in the timeline and begin work on the next frame.


![flowdiagram](https://i.imgur.com/ivDmzPu.png)


The diagram shows the four main elements of the current IK solver.
These elements are listed below:


* [PD Control](https://github.com/osudrl/cassie-trajectory-editor/blob/selection-docs/WRITEUP.md#pd-control)
* [Cleanup](https://github.com/osudrl/cassie-trajectory-editor/blob/selection-docs/WRITEUP.md#cleanup)
* [Target Cutoff](https://github.com/osudrl/cassie-trajectory-editor/blob/selection-docs/WRITEUP.md#target-cutoff)
* [Setup](https://github.com/osudrl/cassie-trajectory-editor/blob/selection-docs/WRITEUP.md#ik-setup)

#### PD Control


If it was possible to set a body's position, there would not be a need for an IK solver.
Instead of allowing direct control of body positions, [MuJoCo](http://www.mujoco.org/) allows the solver to apply [external forces](http://www.mujoco.org/book/reference.html#mjcb_control) to the body.


The PD controller drags the body to the target position by [applying these external forces](https://github.com/osudrl/cassie-trajectory-editor/blob/0dbf44c7536c35cd1c7d0dfab21b6e0a6ace8941/src/pdik.c#L30)..
The direction of the force vector is always toward the target position, but its magnitude is set by the output of the PD controller.
For this PD controller,  the P term is proportional to the body's current distance to the target.
The D term is proportional to the current velocity of the body, slowing it down as it approaches the target.


Implementing a PD controller requires a tuning the constants for the P and D terms.
Increasing in either constant helps the body get to target faster.
Yet increasing these parameters also increases the instability of the simulation.
Much more than the P term, increasing the weighting of the D term caused significant simulation instability.


I ran [automated testing](https://github.com/osudrl/cassie-trajectory-editor/tree/automatedrop) overnight to determine the most efficient set of constants.
The [test script](https://github.com/osudrl/cassie-trajectory-editor/blob/automatedrop/auto.py) tested different weightings for the two terms.
The tester plotted simulation steps (dependent variable) as a function of Kp and Kd.
The script generated data for two scenarios: [lifting the right foot up](https://github.com/osudrl/cassie-trajectory-editor/blob/87ed7f0df94cba1e70309e44e64a87882f006453/auto-liftleg.csv) and [swinging the right foot out](https://github.com/osudrl/cassie-trajectory-editor/blob/87ed7f0df94cba1e70309e44e64a87882f006453/auto-swingleg.csv).
[This script](https://github.com/osudrl/cassie-trajectory-editor/blob/87ed7f0df94cba1e70309e44e64a87882f006453/3dplot.py) plotted these data sets in three dimensions.


<!---https://i.imgur.com/Hbpuxzb.png-->


Kp (Lift) | Kp (Swing)
--- | ---
![lift](https://i.imgur.com/NJNvOV6.png) | ![swing](https://i.imgur.com/ScS2J86.png)


The above set of graphs emphasizes the Kp constant's effect on the solve time.
Increasing Kp cuts down on the computation time, but this effect becomes less pronounced for large Kp values.


Kd (Lift) | Kd (Swing)
--- | ---
![lift](https://i.imgur.com/Ez0qpNy.png) | ![swing](https://i.imgur.com/Hbpuxzb.png)


This next set of graphs shows the 3d plot from an angle which emphasizes the Kd constant's effect.
In general, simulation steps decrease as Kd increases.
But values in the 75-90 cause the simulation instability for the swing leg perturbation, as shown by the sharp divergence upward.
Even values in the 40-60 range cause the solver to fail if Kp was quite large.


I chose the best pair of constants by their ability to consistently converge on the solution, regardless of the perturbation.
Selected constants should err on the side of robustness.


The [default Kp and Kd](https://github.com/osudrl/cassie-trajectory-editor/blob/0dbf44c7536c35cd1c7d0dfab21b6e0a6ace8941/src/ik.c#L106:L107)values for the tool are 480 and 30, although these values could be tuned more accurately with further testing.


#### Cleanup

<!---https://imgur.com/a/GuL7v5K-->

When solving inverse kinematics with the above PD control method, the PD controller only apllies forces to single leg.
While PDIK is being performed, the pelvis is held in place with a spring (`m->qpos_spring`).
However, the other leg does not just stay in place while appling forces to the primary leg.
While the primary leg is being pushed around, the other leg diverges from it staring position.


Normally, transformations are performed on a leg while it is the air, so the other leg is on the ground.
While a leg is standing on the ground, the springs on the cassie leg deflect and store energy.
Therefore, steps in simulation attempting to put the parimary leg in the desired positional also allow the other leg to release this stroed energy, moving this leg out of position.

This issue caused a bug with the IK solver, where dragging a leg around would couse the other leg to move out of position by accident.
The solution saves the initial pose of the robot before inittiaitng any solver steps.
After the solver finishes moving the primary leg, the other leg's joint positions are reset to the initial state.

This phase of the IK solver is called the cleanup phase because the PDIK sovler exits with a valid solution for the primary leg, but causes unwanted side effects on the other leg.
As far as I know, joint modifications on the other leg are unwanted, and these positions should be reset.

<!--- issues #18 and #14 -->


#### Target Cutoff

Because the positions of bodies within MuJoCo are floating point numbers, the body's position after a PDIk step will never be exactly equal to the target postion.
There needs to be a range fo distances to the target within whic the solver decides the IK is "close enough" and ther will not be a signficant differntce in the resulting trajectory if the solver stops here.

The tests in the PD-control section used 0.00005 meters as the cut off, the tool's default cutoff fro solving IK is 1mm.
Visually, there is not much difference betweena solution trajectory with .1mm and 1mm as the IK solver cut off, and these trajectories are planned to bbe used to seed a RL contrller, not use as raw feed foward trajectory.
Furthermore, a trajectory can be refined further using the refiend command if futher accuracy is needed for a certain transformation.

#### IK Setup


<!---https://imgur.com/a/KIEMrxg-->

<!---https://imgur.com/a/bUZJipk-->




### Dead End Solutions



#### Complete Seeding



#### Library of Poses



# Conclusion



# Contact


This tool and documentation page was written by [Kevin Kellar](https://github.com/kkevlar) for use within the [Dynamic Robotics Laboratory](http://mime.oregonstate.edu/research/drl/) at Oregon State University. 
For issues, comments, or suggestions about the tool or its documentation, feel free to [contact me](https://github.com/kkevlar) or [open a GitHub issue](https://github.com/osudrl/cassie-trajectory-editor/issues?utf8=%E2%9C%93&q=label%3Adocs+).








