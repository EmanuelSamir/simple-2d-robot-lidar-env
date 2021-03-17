# 2D Robot and Lidar Simulation

**2D Robot Environment - OpenAI Gym**
*This is a simple simulation for a 2d robot and lidar.*

__Observation Space:__ 
- Robot position (X,Y) coordinates.
- LiDAR measurements (Retrieved by the simulation space).
__Action Space:__ 
- Robot velocities (X,Y).
__Reward:__ 
- Euclidean distance between current and goal positions.

*The episode ends based on three conditions: the robot reaches its desired position (using an \epsilon parameter), at 200 steps, crashes with an obstacle.

## TODO
- [X] Environment creation
- [ ] Define reward conditions
- [ ] Integration with ROS
- [ ] Master merge

