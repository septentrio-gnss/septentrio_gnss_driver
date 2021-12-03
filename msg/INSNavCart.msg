# INSNavCart block 
# Block_Number 4225

std_msgs/Header header

# SBF block header including time header
BlockHeader block_header

uint8       gnss_mode
uint8       error
uint16      info
uint16      gnss_age
float64     x
float64     y
float64     z
uint16      accuracy
uint16      latency
uint8       datum
#uint8       reserved
uint16      sb_list

# INSNavCartPosStdDev sub-block definition:
# If the Position StdDev field is 1 then this sub block is available:
float32     x_std_dev
float32     y_std_dev
float32     z_std_dev

# INSNavCartPosCov sub-block definition:
# If the Position Cov field is 1 then this sub block is available:
float32     xy_cov
float32     xz_cov
float32     yz_cov

# INSNavCartAtt sub-block definition:
# If the Attitude field is 1 then this sub block is available:
float32     heading
float32     pitch
float32     roll

# INSNavCartAttStdDev sub-block definition:
# If the Attitude StdDev field is 1 then this sub block is available:
float32     heading_std_dev
float32     pitch_std_dev
float32     roll_std_dev

# INSNavCartAttCov sub-block definition:
# If the Attitude Cov field is 1 then this sub block is available:
float32     heading_pitch_cov
float32     heading_roll_cov
float32     pitch_roll_cov

# INSNavCartVel sub-block definition:
# If the Velocity field is 1 then this sub block is available:
float32     vx
float32     vy
float32     vz

# INSNavCartVelStdDev sub-block definition:
# If the Velocity StdDev field is 1 then this sub block is available:
float32     vx_std_dev
float32     vy_std_dev
float32     vz_std_dev

# INSNavCartVelCov sub-block definition:
# If the Velocity Cov field is 1 then this sub block is available:
float32     vx_vy_cov
float32     vx_vz_cov
float32     vy_vz_cov