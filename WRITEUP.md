

# Introduction

<img align="right" src="https://i.imgur.com/rlcpkPP.gif" width="350"> 

The Cassie Trajectory Editor aids in designing reference trajectories for the [Cassie robot](http://www.agilityrobotics.com/robots/). 
The trajectories serve as targets in the [reinforcement learning process](https://arxiv.org/abs/1803.05580). 
Both physical and [simulated Cassie](https://github.com/osudrl/cassie-mujoco-sim) can learn different walking gaits through this process.

The editor streamlines the design process for creating learned Cassie controllers.
The learning process trains a controller to mimic a reference behavior.
However, it only rewards a controller if it can keep the robot upright.
In this way, researchers can teach Cassie to mimic any behavior without falling over.
This process, however, requires a reference trajectory featuring the desired behavior.
Cassie trajectory editor solves this problem, by allowing researchers to design and export these reference trajectories.
As a result, it serves a critical role in the reinforcement learning workflow.

The tool helps users manipulate a trajectory, which is represented as a timeline of robot poses.
The user chooses to initialize this timeline with a either full trajectory or a single pose.
When initialized with a trajectory, editor's tools help tweak gait parameters such as step height or foot speed.
If instead the user starts with a single pose, the tool can help create behaviors such as walking, crouching, and hopping.
Finally, the user exports the finished trajectory for use in reinforcement learning process.



# Research Learning


## Modification Tools

<!--https://imgur.com/a/EUN9YRz-->
<!--https://imgur.com/a/EUN9YRz-->

<img align="right" src="https://user-images.githubusercontent.com/10334426/44121073-5f04952c-9fd3-11e8-99fe-6454cc62a26d.png" width="375">

### The Scaling Problem

At the start of this project, I wasn't sure what transformation tools the editor would need.
The editors' first modification tool allowed the user to drag a body in real time while solving spline interpolation.
Yet there was no need to create trajectories based on the mouse's real time position.


The editor allows perturbations by dragging and dropping a node along the trajectory.
Once the user performs a perturbation, the nearby nodes need to move to maintain continuity along the trajectory.



<img align="right" src="https://i.imgur.com/DaJm0NS.png" width="375">

<br></br>

### A-Scaling

The most obvious smoothing method scales the initial perturbation for nearby nodes (A-Scaling).
This method solves every pose as if the user dragged the body at this pose.
But for these poses, the distance shortens as the frames get further and further from the actual perturbation frame.

<br></br>

### Scaling Parameters

I implement this effect using the Gaussian distribution (bell curve).
At the center of the distribution, the scale factor is 1, because the root frame receives the full perturbation.
The scale factor tapers off to zero at the ends of the curve, ensuring pose continuity across the entire trajectory.

Gaussian smoothing needs a width and height.
The standard deviation defines the width of the distribution.
Modifying the width will cause the transformation to affect more/less frames.

At first, I assumed there was no reason to allow the user to tweak the distribution height.
Increasing the max scale factor above 1 would cause transformations to extend beyond the mouse.
But by capping the filter scaling factor at 1, increasing the height will form a sort of mesa effect.
The full transformation applies to many frames, defined by the distribution width.

### B-Scaling vs A-Scaling

A-Scaling | B-Scaling
--- | ---
![ascaling](https://i.imgur.com/DaJm0NS.png) | ![bscaling](https://user-images.githubusercontent.com/10334426/44121302-40330c5e-9fd4-11e8-9339-2b9ba7286024.png)

A-Scaling only considers the final position of the node compared to its initial position.
In contrast, I use a different scaling strategy, B-Scaling.
B-Scaling compares each node's own staring position to the mouse's end point.
As a result, each node undergoes a unique translation, both in direction and magnitude.

B-Scaling transformations allows nodes to converge on a target.
A user can use B-Scaling to [hold a body in place](https://i.imgur.com/ynlZpe3.gif) for a set of frames.
Increasing the height of the Gaussian distribution causes many nodes move all the way to the mouse's ending position.
The user can use these tools to hold a body at a single position for some time.


## Inverse Kinematics

<img align="right" src="https://i.imgur.com/2nrSmNf.png" width=250 > 

[Inverse kinematics](https://medium.com/unity3danimation/overview-of-inverse-kinematics-9769a43ba956) is finding a set of joint angles such that the position of a body ends up at a target.
If the user drags the robot's foot to a new position, we need to know what hip, knee and tarsus joints will get the foot there.
Every time the user drags a set of nodes (with some exceptions), the program solves inverse kinematics.

If the user were to make the above transformation, the foot is in a new position for each frame for most of the timeline.
The solver knows the target position for each discreet pose.
But only inverse kinematics can determine what joint angles, will get the foot to its target.


The tool implements an inverse kinematics solver which relies on MuJoCo physics simulation.
At a high level, the simulation first initializes the robot's pose.
Next, a [PD controller](http://robotic-controls.com/learn/programming/pd-feedback-control-introduction) applies external forces to the body, pushing it towards the target position.
The solver accepts the current pose as a solution when the body's position is within the accepted error range.
Once accepted, we store the joint positions in the timeline and begin work on the next frame.


![flowdiagram](https://user-images.githubusercontent.com/10334426/44120238-a37877da-9fd0-11e8-86ca-460781af76b6.png)


The diagram shows the four main elements of the current IK solver.
These elements are listed below:


* [PD Control](https://github.com/osudrl/cassie-trajectory-editor/blob/selection-docs/WRITEUP.md#pd-control)
* [Cleanup](https://github.com/osudrl/cassie-trajectory-editor/blob/selection-docs/WRITEUP.md#cleanup)
* [Setup](https://github.com/osudrl/cassie-trajectory-editor/blob/selection-docs/WRITEUP.md#ik-setup)

### PD Control


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


The [default Kp and Kd](https://github.com/osudrl/cassie-trajectory-editor/blob/0dbf44c7536c35cd1c7d0dfab21b6e0a6ace8941/src/ik.c#L106:L107) values for the tool are 480 and 30, although these values could be tuned more accurately with further testing.

<img align="right" src="https://i.imgur.com/SzfOnoD.gif" width="300"> 

### Cleanup

<!---https://imgur.com/a/GuL7v5K-->



When solving inverse kinematics with the above PD control method, the PD controller only applies forces to single leg.
While PDIK is being performed, `m->qpos_spring` hold the pelvis in place.
But the solver does not apply any forces to the other leg.
So while the solver pushes the primary leg towards the target, the other leg diverges from it staring position.

In most cases, the user will want to transform a leg while it is the air.
Yet lifting a leg in the air requires Cassie to firmly plant the other leg on the ground.
While standing on the ground, the springs in the Cassie leg deflect and store energy.
The IK solver allows the other leg to release this stored energy, moving this leg out of position.

This issue caused a bug with the IK solver, where perturbing a leg would cause the other leg to move out of position.
To fix this bug, the solver saves the initial pose of the robot before initiating any solver steps.
After finishing moving the primary leg, the other leg's joints are reset to their initial state.

I named this phase"cleanup" because once the PDIK solver exits, it causes unwanted side effects on the other leg.
As far as I know, joint modifications on the other leg are unwanted, and these positions should be reset.

<!--- issues #18 and #14 -->



### IK Setup


<!---https://imgur.com/a/KIEMrxg-->

<!---https://imgur.com/a/bUZJipk-->


<img align="right" src="https://user-images.githubusercontent.com/10334426/44125251-5c2ccc6c-9fe6-11e8-9641-b55f9eeeeec0.png" width="300"> 

#### Full Seeding

The current solver optimizes the process by seeding the last (solved) frame's pose for subsequent calculation.
This seed will be much closer to the target than this frame's initial position.
As a result, the solver needs fewer simulation cycles to get the body to the target.
Furthermore, a shorter distance allows the solver to use larger PD constants without risking simulation instability.

There is a disadvantage: **complete** seeding finds a "band" of solutions.
The robot poses are continuous within the "solved" frames.
Yet the edges of the solved frames diverge from the original trajectory.
Ideally, the solver only slightly modifies the poses at the tail ends of the smoothed perturbation.
But full seeding results in significant discontinuities in the "solved" trajectory.

<!---https://imgur.com/a/pPfTpM3-->


#### Seeding vs No Seeding

<img align="right" src="https://i.imgur.com/SKCPJfL.gif" width="300"> 

Not seeding previous solutions forces each solved frame to respect the initial pose of the robot before the any perturbation.
The solution maintains continuity both within the solution and with the starting trajectory at the ends. 
However, seeding the previous IK solution allows the solution trajectory to diverge. 
The error doesn't diverge, but joints diverge from their initial positions.
When using full seeding, the initial pose for a frame has no impact on the solved pose for that frame.

#### Solution: Partial Seeding

Seeding the last solution causes obvious problems, but allowing the solver to start "from scratch" for each frame takes so much longer.
So the current solver makes a compromise between the two approaches.
When the IK solver is called, it partially seeds the last solution's qposes.
The setup phase will add 95% of the previous solution to the initial position of for this frame.
Partial seeding speeds up computation because the body will start much closer to the target.
Yet the starting pose has enough of an effect that the solution trajectory will not diverge.

0% | 100% | 95%
--- | --- | ---
Computation time: **44.7 sec** | Computation time: **1.7 sec** | Computation time: **1.7 sec**
![0](https://i.imgur.com/ODM9GNz.gif) | ![100](https://i.imgur.com/SKCPJfL.gif) | ![95](https://i.imgur.com/Mof3qL0.gif)



# Conclusion



# Contact


This tool and documentation page was written by [Kevin Kellar](https://github.com/kkevlar) for use within the [Dynamic Robotics Laboratory](http://mime.oregonstate.edu/research/drl/) at Oregon State University. 
For issues, comments, or suggestions about the tool or its documentation, feel free to [contact me](https://github.com/kkevlar) or [open a GitHub issue](https://github.com/osudrl/cassie-trajectory-editor/issues?utf8=%E2%9C%93&q=label%3Adocs+).








