# Hierarchical Planning for Indoor Mobile Robots in a Dynamical Environment
Autonomous Mobile Robots are gradually becoming a reality in many applications. The most
important Decision Making module that goes into it is Planning and Navigation. This work
tries to implement a Hierarchical Planning algorithm to navigate around Dynamic Obstacles.
First, a Global Planner operates on a low resolution Occupancy Grid, using the A* algorithm,
to create a high level plan in the form of waypoints that the bot should follow. A Local Planner
then operates within the high resolution perceptive field around the bot to plan for
uncertainties in the present state. The Local Planner uses an Ego-Graph approach to generate
locally optimal paths described by Motion Primitives that are easy to follow and optimized for
various cost functions like cross track error and curvature error. Dynamic Obstacles are
handled by forward propagating the current motion of said obstacles and then performing a
collision check through time. The aim is to reach the goal moving from waypoint to waypoint
while avoiding static and dynamic obstacles. Algorithms to reject unnecessary waypoints are
also used. All of the above is simulated and tested using a Differential Drive based Indoor
Mobile Robot. [Read Report](ED18B027_Final_Report.pdf)

## TurtleBot3 World Static
<img src="results/world_static_gazebo.gif" width="400"/><img src="results/world_static_paths.gif" width="400"/>

## ED Floor Static
<img src="results/ed_static_gazebo.gif" width="400"/><img src="results/ed_static_paths.gif" width="400"/>

## ED Floor with Dynamic Obstacles
<img src="results/ed_dyn_gazebo.gif" width="400"/><img src="results/ed_dyn_paths.gif" width="400"/>

## Implementation Flow Diagram
![image](https://github.com/Rohitth007/hierarchical_planner_dyn_obstacles/assets/64144419/910e61cc-ec08-49f5-9b8b-bb826554e871)

## Collision Checking in Time
![image](https://github.com/Rohitth007/hierarchical_planner_dyn_obstacles/assets/64144419/0cf11159-78b6-471c-a783-a4b9a364076a)

## Dynamic Obstacle Tracking using [obstacle_detector](https://github.com/tysik/obstacle_detector)
![image](https://github.com/Rohitth007/hierarchical_planner_dyn_obstacles/assets/64144419/88984cec-95f3-4542-8efa-30eaa6ae8fab)
