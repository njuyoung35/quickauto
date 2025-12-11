# Longitudinal control message.
# - Values are in the base_link frame in X axis
# - 양수 : 앞으로
# - 음수 : 뒤로

# Time this message was created
builtin_interfaces/Time stamp 

# Time this configuration state is to expected to be achieved in (optional)
builtin_interfaces/Time control_time 

# Desired vehicle velocity [m/s]
float32 velocity 

# Desired vehicle acceleration [m/s^2]
float32 acceleration 

# Desired vehicle jerk [m/s^3]
float32 jerk 

bool is_acceleration_defined 

bool is_jerk_defined 
