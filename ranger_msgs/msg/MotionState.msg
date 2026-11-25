uint8 MOTION_MODE_DUAL_ACKERMAN = 0
uint8 MOTION_MODE_PARALLEL = 1
uint8 MOTION_MODE_SPINNING = 2
uint8 MOTION_MODE_SIDE_SLIP = 3

########################################

std_msgs/Header header

# motion state
uint8 motion_mode


# Appended fields: what the base reports about its motion in this cycle, as received, and the
# geometry the driver's odometry integrated with. Angles in rad (left positive, as measured in
# oblique mode), speeds in m/s, wheel values in the SDK's order (angle_5..8, speed_1..4).
uint8 ODOMETRY_GEOMETRY_DUAL_ACKERMAN = 0
uint8 ODOMETRY_GEOMETRY_PARALLEL = 1
uint8 ODOMETRY_GEOMETRY_SPINNING = 2
uint8 ODOMETRY_GEOMETRY_NONE = 3

bool mode_changing
float64 linear_velocity
float64 angular_velocity
float64 steering_angle
float64[4] wheel_angles
float64[4] wheel_speeds
uint8 odometry_geometry
