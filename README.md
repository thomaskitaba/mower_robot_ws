# mower_robot_ws
# mower_robot_ws
# mower_robot_ws
# mower_robot_ws
# mower_robot_ws
Thanks..
Automatic Lawn Mower Robot
By: Thomas Kitaba
Instructors: Dr Adane Letta, Debela Desalegn
Submission Date: 6/01/2025 GC
Addis Ababa University Institute of Technology
School of Information Technology and
Engineering
Department of Artificial Intelligence
2
Table of Content
1. Motivation / Problem Statement
2. Objective
3. Application
4. Physical look and components of the robot
5. Development of the Automated Lawn Mower Robot
5.1. Environment Mapping and Localization
5.1.1 Mapping
5.1.2 Localisation
5.2. Navigation and Obstacle Avoidance
5.3 Coverage Path Planning
6. Mowing and Simulation
1
1. Motivation / Problem Statement
This days our country is going through an astonishing road-side
renovation, which encompasses almost 100 kilometer cities main road.
Where grasses are being planted in many places different areas.
Maintaining lawns in such a residential, commercial, and public areas is
labor-intensive, time-consuming, and often inconsistent, especially over
large or irregular terrains. Manual mowing requires regular effort, and
traditional mowers pose safety and environmental concerns. There is a
growing need for a smart, autonomous solution that can perform lawn
mowing efficiently, safely, and with minimal human intervention.
2. Objective
To develop an autonomous robot capable of mowing lawns efficiently
while navigating uneven or dynamic outdoor environments. The robot
will:
 Map and localize itself within the lawn boundary.
 Plan and execute complete and optimized coverage paths.
 Dynamically detect and avoid obstacles such as pets, trees, or garden
furniture.
 Maintain safety and precision in different weather and terrain
conditions.
3. Application
The lawn mower can be used in different environments for different
mowing purposes, the application areas are provided below.
Residential Use:
Automate routine lawn maintenance for homeowners, increasing
convenience and safety.
Commercial Landscaping:
Deploy across parks, golf courses, and office lawns to save labor and
ensure consistent maintenance.
2
Sustainable Lawn Care:
Reduce emissions and noise pollution by using electric autonomous
mowers.
Smart City Integration:
Coordinate with IoT systems for scheduled mowing, weather adaptation,
and remote monitoring.
4. Physical look and components of the robot
Actuators:
1. Chassis: Should be rugged and weather-resistant with appropriate
ground clearance.
2. Wheels: Differential drive (2-wheel + caster) is common for
maneuverability; consider 4WD for rough terrain.
3. Blades: Safety shielded, electric motor-driven; must automatically
stop on lift/tilt.
Sensors:
 Lidar or Ultrasonic: For obstacle detection.
 IMU (Inertial Measurement Unit): For stability and orientation.
 Wheel Encoders: For odometry (distance tracking).
 GPS + RTK: For accurate outdoor localization (especially in large
areas).
 Camera (optional): For advanced perception or visual SLAM.
Processing:
Microcontroller + Companion Computer (e.g., Arduino + Raspberry Pi or
Jetson Nano) Must support real-time sensor fusion, path planning, and
actuation.
3
5. Development of the Automated Lawn Mower Robot
5.1 Environment Mapping and Localization
5.1.1 Mapping:
SLAM:
GMapping: Useful for structured 2D environments such as fenced
gardens.
Cartographer: Offers real-time loop closure, beneficial for large or
repetitive outdoor spaces.
Selected Mapping Method: Cartographer
Why: since the lawn mower might need to recharge and it is expected to
work even in large areas for long hours.
5.1.2 Localisation:
AMCL (Adaptive Monte Carlo Localization): Works well in known yard
layouts with static features.
EKF/UKF (Extended/Unscented Kalman Filter): More suitable for GPS-
integrated systems or variable environments with terrain undulations.
Selected Localisation method: EKF/UKF (Extended/Unscented Kalman
Filter):
5.2. Navigation and Obstacle Avoidance
Potential Algorithms choices:
Global Path Planning:
Plans the route to a target using map knowledge.
Candidate Algorithms:
 A* or Dijkstra's Algorithm: For p redefining efficient paths in well-
bounded gardens.
 RRT (Rapidly-exploring Random Tree): Useful in dynamic or highly
cluttered outdoor spaces.
4
Local Path Planning:
React to real-time obstacles during movement.
 Dynamic Window Approach (DWA): Reacts to real-time obstacles like
pets or toys.
 Teb Local Planner: Maintains smooth and curved paths around
curved flower beds or trees.
Selected Navigation and Obstacle Avoidance Algorithm:
Timed Elastic Band (TEB) Local Planner
Why:
The Timed Elastic Band (TEB) Local Planner excels in generating smooth,
time-optimized, and feasible trajectories in real time. It considers the
robot’s kinematics and dynamics, which is crucial for a lawn mower
moving through narrow paths, curves, and around static and dynamic
obstacles like trees, flower beds, pets, and toys.
- Smooth motion: Essential for consistent mowing coverage.
- Obstacle avoidance: Reacts well to both static and dynamic obstacles.
- Speed & curvature optimization: Avoids jerky movements that can
damage grass or miss spots.
5.3 Coverage Path Planning
Systematically cover the entire lawn with minimal overlap and no gaps.
The robot will comprise of hybrid coverage planner that the following
three algorithms:
4.3.1 Default mode:
This mode is the default mode, where the environment is expected to be
small or medium and the shape of the field is rectangular.
Chosen Algorithm: Boustrophedon Coverage
Why: Boustrophedon Coverage: Ideal for rectangular lawns; performs
systematic back-and-forth mowing, it also Performs efficient “zig-zag”
mowing much like a farmer plowing a field.
Highly efficient in large rectangular zones or after decomposing
irregular areas into smaller regular sections.
5
4.3.2 Expert mode:
This mode will be activated by a button when the train is complex
with irregular shape and with different types of obstacles like trees,
Candidate Algorithms: This mode will be implemented using STC
(Spanning Tree Coverage) algorithm.
Why: Spanning Tree Coverage (STC) is designed to systematically cover
all free space even in irregular, obstacle-filled environments — which is
common in real gardens with trees, furniture, or flower beds.
STC Efficiently works around obstacles, it guarantees Full coverage:
Ensures every accessible patch is mowed.
STC Can be adapted to grid-based, sensor-based, or map-based
implementations. It is the best suited for real-world, non-uniform lawns.
6. Mowing and Simulation
Simulation Environment (e.g., Gazebo):
 Visualize mowed vs. unmowed areas using color-coded patches.
 Simulate blade coverage with virtual contact zones.
 Blade Control Logic: Simulate blade activation only within grass
zones to optimize energy use.
 Implement safety cut-off mechanisms if obstacles are detected.