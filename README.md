# 2D Robot and Lidar Simulation

**2D Robot Environment - OpenAI Gym**

*This is a simple environment for reinforcement learning applications*

There are two examples in branches. 
- for mapless navigation task, discussed in an extended abstract.
- for mapless exploration, discussed in submitted paper.

Publication will be soon available

__Observation Space:__ 
- Robot position (X,Y) coordinates.
- LiDAR measurements (Retrieved by the simulation space).

__Action Space:__ 
- Robot velocities (X,Y).

__Reward:__ 
Navigation
- Euclidean distance between current and goal positions.
- Collision
Exploration
- Found object
- Collision


*The episode ends based on three conditions: the robot reaches its desired position (using an ε parameter), at 600 steps, crashes with an obstacle.*

