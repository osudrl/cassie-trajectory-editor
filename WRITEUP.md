

# Introduction


The Cassie Trajectory Editor aids in designing and exporting reference walking trajectories, specifically for the [Cassie robot](http://www.agilityrobotics.com/robots/). The exported trajectory serves as a reference in the [reinforcement learning process](https://arxiv.org/abs/1803.05580) such that the trajectory designed in this tool may be learned on a [simulated Cassie](https://github.com/osudrl/cassie-mujoco-sim) and the physical robot.


The tool *will allow* the user to either [initialize](https://github.com/osudrl/cassie-trajectory-editor/blob/docs/README.md#initialization) the timeline with a single pose or a full trajectory. During the editing process, the tool helpes the developer visualize changes to the initial timeline. After designing the walking gait, the user exports the finished trajectory to be used in the learning process.

<img src="https://i.imgur.com/rlcpkPP.gif" width="400"> 

# Methods and Results



## User Experience



### Keybinds



### Installation



### Calculation Delays



## Inverse Kinematics


https://medium.com/unity3danimation/overview-of-inverse-kinematics-9769a43ba956


[Imgur](https://i.imgur.com/2nrSmNf.png)

### Preferred Solution



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


This tool and documentation page was written by [Kevin Kellar](https://github.com/kkevlar) for use within the [Dynamic Robotics Laboratory](http://mime.oregonstate.edu/research/drl/) at Oregon State University. For issues, comments, or suggestions about the tool or its documentation, feel free to [contact me](https://github.com/kkevlar) or [open a GitHub issue](https://github.com/osudrl/cassie-trajectory-editor/issues).








