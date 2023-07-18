# INSNavGeod block 
# Block_Number 4226

std_msgs/Header header

# SBF block header including time header
BlockHeader block_header

uint8       gnss_mode
uint8       error
uint16      info
uint16      gnss_age   # 0.01 s
float64     latitude   # rad
float64     longitude  # rad
float64     height     # m (ellipsoidal)
float32     undulation # m
uint16      accuracy   # 0.01 m
uint16      latency    # 0.0001 s
uint8       datum
#uint8       reserved
uint16      sb_list

# INSNavGeodPosStdDev sub-block definition:
# If the Position StdDev field is 1 then this sub block is available:
float32     latitude_std_dev  # m
float32     longitude_std_dev # m
float32     height_std_dev    # m

# INSNavGeodPosCov sub-block definition:
# If the Position Cov field is 1 then this sub block is available:
float32     latitude_longitude_cov # m^2
float32     latitude_height_cov    # m^2
float32     longitude_height_cov   # m^2

# INSNavGeodAtt sub-block definition:
# If the Attitude field is 1 then this sub block is available:
float32     heading # deg
float32     pitch   # deg
float32     roll    # deg

# INSNavGeodAttStdDev sub-block definition:
# If the Attitude StdDev field is 1 then this sub block is available:
float32     heading_std_dev # deg
float32     pitch_std_dev   # deg
float32     roll_std_dev    # deg

# INSNavGeodAttCov sub-block definition:
# If the Attitude Cov field is 1 then this sub block is available:
float32     heading_pitch_cov # deg^2
float32     heading_roll_cov  # deg^2
float32     pitch_roll_cov    # deg^2

# INSNavGeodVel sub-block definition:
# If the Velocity field is 1 then this sub block is available:
float32     ve # m/s
float32     vn # m/s 
float32     vu # m/s

# INSNavGeodVelStdDev sub-block definition:
# If the Velocity StdDev field is 1 then this sub block is available:
float32     ve_std_dev # m/s
float32     vn_std_dev # m/s
float32     vu_std_dev # m/s

# INSNavGeodVelCov sub-block definition:
# If the Velocity Cov field is 1 then this sub block is available:
float32     ve_vn_cov # m^2/s^2
float32     ve_vu_cov # m^2/s^2
float32     vn_vu_cov # m^2/s^2   
