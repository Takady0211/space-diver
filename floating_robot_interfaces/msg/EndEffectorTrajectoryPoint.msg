# Point in 6DOF space. 
geometry_msgs/Pose pose
geometry_msgs/Twist twist
geometry_msgs/Accel accel
builtin_interfaces/Duration time_from_start

# Path following way to get point.
int32 path_generation_id
int32 IGNORE_TRAJECTORY = -1
int32 STRAIGHT_LINE_PATH = 0

# Velocity following way to get point.
int32 time_scaling_id
int32 CONSTANT_TIME_SCALING = 0
int32 TRIANGLE_TIME_SCALING = 1
int32 TRAPEZOIDAL_TIME_SCALING = 2
int32 POLY3_TIME_SCALING = 3
int32 PLOY5_TIME_SCALING = 5