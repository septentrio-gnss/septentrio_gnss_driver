# ExtSensorMeas block 
# Block_Number 4050

std_msgs/Header header

# SBF block header including time header
BlockHeader block_header

uint8   n
uint8   sb_length

# ExtSensorMeasSet
uint8[]   source
uint8[]   sensor_model
uint8[]   type
uint8[]   obs_info

#ExtSensorMeasAcceleration
float64 acceleration_x
float64 acceleration_y
float64 acceleration_z

#ExtSensorMeasAngularRate
float64 angular_rate_x
float64 angular_rate_y
float64 angular_rate_z

#ExtSensorMeasVelocity
float32 velocity_x
float32 velocity_y
float32 velocity_z
float32 std_dev_x
float32 std_dev_y
float32 std_dev_z

#ExtSensorMeasInfo
float32 sensor_temperature # deg C

#ExtSensorMeasZeroVelocityFlag
float64 zero_velocity_flag