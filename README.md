# 2D Robot and Lidar Simulation

**2D Robot Environment - OpenAI Gym**
*This is a simple simulation for a 2d robot and lidar.*

Observation Space: 
- Robot position (X,Y) coordinates.
- LiDAR measurements (Retrieved by the simulation space).
Action Space: 
- Robot velocities (X,Y).
Reward: 
- Euclidean distance between current and goal positions.

*The episode ends based on three conditions: the robot reaches its desired position (using an \epsilon parameter), at 200 steps, crashes with an obstacle.

## TODO
- [X] Environment creation
- [ ] Define reward conditions
- [ ] Integration with ROS
- [ ] Master merge

