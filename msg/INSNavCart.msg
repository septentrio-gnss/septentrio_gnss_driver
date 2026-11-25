# INSNavCart block 
# Block_Number 4225

std_msgs/Header header

# SBF block header including time header
BlockHeader block_header

uint8       gnss_mode
# Type of PVT solution in bits 0-3 of gnss_mode. Bits 4-6 are reserved and bit 7
# carries an independent flag, so gnss_mode must be masked with MODE_TYPE_MASK
# before it is compared against these values. 9, 11 and 12 are reserved by the
# SBF spec.
#------------------------------------------------------------------------------
uint8 MODE_TYPE_MASK             = 15
uint8 MODE_NO_PVT                = 0
uint8 MODE_STANDALONE            = 1
uint8 MODE_DGNSS                 = 2
uint8 MODE_FIXED_LOCATION        = 3
uint8 MODE_RTK_FIXED             = 4
uint8 MODE_RTK_FLOAT             = 5
uint8 MODE_SBAS                  = 6
uint8 MODE_MOVING_BASE_RTK_FIXED = 7
uint8 MODE_MOVING_BASE_RTK_FLOAT = 8
uint8 MODE_PPP                   = 10
# Independent flag. Bit 7 is set when GNSS-based attitude is not requested by the
# user; in that case all the other bits are zero.
uint8 MODE_FLAG_GNSS_ATTITUDE_NOT_REQUESTED = 128
#------------------------------------------------------------------------------
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