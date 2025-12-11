# Lateral control message for Ackermann-style platforms
# - 양수 : 좌측
# - 음수 : 우측

# Time this message was created
builtin_interfaces/Time stamp 

# Time this configuration state is to expected to be achieved in (optional)
builtin_interfaces/Time control_time 

# Desired angle of steering tire [rad]
float32 steering_tire_angle 

# Desired rate of change of the steering tire [rad/s]
float32 steering_tire_rotation_angle 

bool is_steering_tire_rotation_rate_defined 
