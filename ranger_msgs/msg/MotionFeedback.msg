# What the base reports about its motion in each driver cycle, as received. Published on
# /motion_feedback with the same header stamp as /odom, /motion_state, /system_state and
# /actuator_state of that cycle. Angles in rad (left positive, as measured in oblique mode),
# speeds in m/s, wheel values in the SDK's order (angle_5..angle_8, speed_1..speed_4).

uint8 ODOMETRY_GEOMETRY_DUAL_ACKERMAN = 0
uint8 ODOMETRY_GEOMETRY_PARALLEL = 1
uint8 ODOMETRY_GEOMETRY_SPINNING = 2
uint8 ODOMETRY_GEOMETRY_NONE = 3

std_msgs/Header header

uint8 motion_mode            # reported motion mode (same value as /motion_state)
bool mode_changing           # firmware flag: a mode switch is in progress
float64 steering_angle       # reported steering angle (motion-state frame)
float64 linear_velocity      # reported linear velocity (motion-state frame)
float64 angular_velocity     # reported angular velocity (motion-state frame)
float64[4] wheel_angles      # measured steering angles of the four wheels
float64[4] wheel_speeds      # measured speeds of the four wheels
uint8 odometry_geometry      # geometry the driver's odometry integrated with in this cycle
