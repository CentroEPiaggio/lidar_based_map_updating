# lidar_based_map_updating
Ros-Packages to update an occupancy grid map with lidar measurements.

Abstract
Long-time operations of autonomous vehicles and mobile robots in logistics and service applications are still a challenge. To avoid a continuous re-mapping, the map can be updated to obtain a consistent representation of the current environment. In this paper, we propose a novel LIDAR-based occupancy grid map updating algorithm for dynamic environments taking into account possible localisation and measurement errors. The proposed approach allows robust long-term operations as it can detect changes in the working area even in presence of moving elements. Results highlighting \modified{map} quality and localisation performance, both in simulation and experiments, are reported.



Requirements:
- A localisation algorithm
- An occupancy grid map 

Algorithm parameters present in parameters.yaml in the folder param:

changed_cells_topic: /changed_cells   name of the changed cells topic
map_topic: /map                       name of the map topic 
processed_map_topic: /processed_map   name of the updated map topic 

map_updater:
  base_frame: base_link               name of the robot base frame name
  map_frame: map                      name of the map frame 
  scan_topic: /scan                   name of the scan topic 
  odom_topic: /odom                   name of the odom topic 

  update_map_online: true

  init_buffers: false

  min_angle_increment: 0.1 #0.5  # expressed in degrees, it determines the considered beams number

  # The pairing distance is taken as a function of the measured range R. In particular, it grows linearly with R until saturation is reached
  # The configurable parameters are: saturation value and the desired pairing distance for two different range measurement
  range_1_pair: 0.2
  pairing_distance_1: 0.15
  range_2_pair: 10.0
  pairing_distance_2: 0.28
  pairing_distance_saturation: 0.3

  buffer_size: 10               # rolling buffer size
  counters_change_threshold: 7  # cell is considered as changed if the corresponding counter is greater than this threshold
  #counters_change_hysteresis: 0

  scan_update_interval: 0.3 #1.0       # min time that has to elapse between two scans processing
  min_linear_displacement: 0.4 #0.5    # min distance that the robot has to travel between two scans processing
  min_angular_displacement: 45.0  # expressed in degrees

  max_beam_range: 18.0 #15.0  # measurements greater than this will be discarded
  max_range_tol: 0.8    # virtual beams will measure no more than "max_beam_range" + "max_range_tol"

  virtual_to_measured_beam_ratio: 1 #3         # determines the number of virtual beams
  min_search_window_halfwidth_degrees: 4.0  # expressed in degrees

  max_trusted_angular_velocity: 12  # expressed in degrees/s
  min_idle_duration: 1.2 #1.5            # min idle state time induced by a quick rotation
  min_idle_distance: 1.6 #1.8

  final_chunk_skip_fraction: 0.02     # final fraction of the beam that is skipped when doing ray casting for anomalous beams analysis
  min_chunk_skip_length: 0.15         # min skipped length
  skip_fraction_false_positive: 0.02  # final fraction of the beam that is skipped when doing ray casting for false positive detection

  N_skip_anomalous: 2 # number of not anomalous beams that has to be skipped near an anomalous beam

  max_anomalous_beam_fraction: 0.8
    
  # For each anomalous point, a search for an occupied cell is performed within a circle centered in the hit point and with radius that grows linearly with the measured range
  range_1_search: 0.0
  radius_1: 0.10
  range_2_search: 10.0
  radius_2: 0.2


Launch folder:
start_map_update.launch --> launch updating node
monitor_consumption.launch --> launch cpu_monitor node
update_bag --> launch updating node for bag file


Input topic for the map updating algoritm:
- scan topic
- map topic 
- odom topic 

Output topic:
- updated map --> /processed_map
- changed_cells_topic --> /changed_cells 
