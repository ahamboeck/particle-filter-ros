# PRO-LAB
This project was developed as part of a course at UAS Technikum Wien.

## Getting Started

### Dependencies
- ROS Noetic
- Ubuntu 20.04
- Gazebo 11.11.0
- Boost 1.71
- C++14

### Installing
- Clone the repository: `git clone [repository-url]`
- Navigate to the project directory: `cd [project-directory]`
- Build the project using catkin: `catkin_make`
- Source the setup script: `source devel/setup.bash`

## Usage
- Launch the simulation: `roslaunch pro_lab simulation.launch`
- To visualize the particle filter in action, open RViz: `rviz`
- Load the RViz configuration provided in the `rviz/config.rviz`.

## Timeline
This section is only for myself to keep track of what I've done during the time working on it.

### 22.04.2024
Accomplished:
- Setup for the Gazebo playground
- Initial configuration of Gazebo sensors and visualization settings

To-do:
- Make setup ready to only implement 4 poses and the particle filter
- Create a map of the playground for localization

### 05.05.2024
Accomplished:
- Started basic implementation of MCL but erased everything
- Knowledge was gained

### 06.05.2024
Accomplished:
- Defined classes `LocalizationHandler`, `Particle`, and `ParticleFilter`.
- Setup basic ROS publishing and subscribing mechanisms.
- Added visualization of particles in RViz using arrows to indicate direction and scaling based on weight.
- Added random particles during the resampling phase to maintain diversity among particle hypotheses.
- Adjusted the visualization to dynamically change the size of arrows based on particle weights, enhancing visual clarity in RViz.
- Implemented the basic structure for the Monte Carlo localization (particle filter), which is working

To-do:
- Integrate advanced sensor models.
- Conduct extensive testing with different scenarios in the simulation to validate the localization under various conditions.
- Document and prepare for project presentation.

## Contributors
- Alexander Hamböck

## License
This project is licensed under the MIT License - see the LICENSE file for details.
