# INSNavCart block 
# Block_Number 4225

std_msgs/Header header

# SBF block header including time header
BlockHeader block_header

uint8       gnss_mode
uint8       error
uint16      info
uint16      gnss_age # 0.01 s
float64     x        # m
float64     y        # m
float64     z        # m
uint16      accuracy # 0.01 m
uint16      latency  # 0.1 ms
uint8       datum    
#uint8       reserved
uint16      sb_list

# INSNavCartPosStdDev sub-block definition:
# If the Position StdDev field is 1 then this sub block is available:
float32     x_std_dev # m
float32     y_std_dev # m
float32     z_std_dev # m

# INSNavCartPosCov sub-block definition:
# If the Position Cov field is 1 then this sub block is available:
float32     xy_cov # m^2
float32     xz_cov # m^2
float32     yz_cov # m^2

# INSNavCartAtt sub-block definition:
# If the Attitude field is 1 then this sub block is available:
float32     heading # deg
float32     pitch   # deg
float32     roll    # deg

# INSNavCartAttStdDev sub-block definition:
# If the Attitude StdDev field is 1 then this sub block is available:
float32     heading_std_dev # deg
float32     pitch_std_dev   # deg
float32     roll_std_dev    # deg

# INSNavCartAttCov sub-block definition:
# If the Attitude Cov field is 1 then this sub block is available:
float32     heading_pitch_cov # deg^2
float32     heading_roll_cov  # deg^2
float32     pitch_roll_cov    # deg^2

# INSNavCartVel sub-block definition:
# If the Velocity field is 1 then this sub block is available:
float32     vx # m/s
float32     vy # m/S
float32     vz # m/s

# INSNavCartVelStdDev sub-block definition:
# If the Velocity StdDev field is 1 then this sub block is available:
float32     vx_std_dev # m/s
float32     vy_std_dev # m/s
float32     vz_std_dev # m/s

# INSNavCartVelCov sub-block definition:
# If the Velocity Cov field is 1 then this sub block is available:
float32     vx_vy_cov # m^2/s^2
float32     vx_vz_cov # m^2/s^2
float32     vy_vz_cov # m^2/s^2