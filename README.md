# DynamicWindowsApproach
A standalone modern C++ DynamicWindowApproach 

![Demo](avoidance_example.gif)

## Note 
The prediction horizon is a very critical parameter to tune. Since the heading cost is computed only on the last pose of the predicted trajectory, the prediction horizon should be long enough to make the robot able to avoid obstacles and point to the goal at the end of the predicted trajectory.
