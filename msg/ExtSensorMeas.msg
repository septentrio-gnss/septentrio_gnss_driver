# ExtSensorMeas block 
# Block_Number 4050

Header      header

# SBF block header including time header
BlockHeader block_header

uint8   n
uint8   sb_length

# ExtSensorMeasSet
uint8   source
uint8   sensor_model
uint8   type
uint8   obs_info

#ExtSensorMeasAcceleration
float64 acceleration_X
float64 acceleration_Y
float64 acceleration_Z

#ExtSensorMeasAngularRate
float64 angular_rate_X
float64 angular_rate_Y
float64 angular_rate_Z

#ExtSensorMeasVelocity
float32 velocity_X
float32 velocity_Y
float32 velocity_Z
float32 std_dev_X
float32 std_dev_Y
float32 std_dev_Z

#ExtSensorMeasInfo
int16   sensor_temperature

#ExtSensorMeasZeroVelocityFlag
float64 zero_velocity_flag