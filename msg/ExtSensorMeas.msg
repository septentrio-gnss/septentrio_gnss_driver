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
float64 acceleration_x # m/s^2
float64 acceleration_y # m/s^2
float64 acceleration_z # m/s^2

#ExtSensorMeasAngularRate
float64 angular_rate_x # deg/s
float64 angular_rate_y # deg/s
float64 angular_rate_z # deg/s

#ExtSensorMeasVelocity
float32 velocity_x # m/s
float32 velocity_y # m/s
float32 velocity_z # m/s
float32 std_dev_x # m/s
float32 std_dev_y # m/s
float32 std_dev_z # m/s

#ExtSensorMeasInfo
float32 sensor_temperature # deg C

#ExtSensorMeasZeroVelocityFlag
float64 zero_velocity_flag