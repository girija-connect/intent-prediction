# Intent Prediction

Human intent prediction for a TurtleBot3 navigating among people (ROS 2 Humble, Ignition Gazebo, Nav2).

> **Status: work in progress.** The project has recently started; nothing here is stable yet.

## Goal
The robot watches nearby people, predicts what each one intends to do (for example approach, cross its path, pass by, walk away or stand still), and adjusts its navigation to stay out of their way.

## Planned work
- Simulation: office world in Gazebo with walking people and a TurtleBot3
- Navigation: map, localization (AMCL) and Nav2 setup
- Perception: detect and track people, estimate speed and direction
- Intent prediction: classify each person's intent and predict their path
- Navigation integration: feed predictions into the Nav2 costmap
- Safety: speed limits and stop zones around the robot
- Evaluation: compare navigation with and without intent prediction

## Requirements
ROS 2 Humble, Nav2, Ignition Gazebo, TurtleBot3 packages.

## License
See [LICENSE](LICENSE).
