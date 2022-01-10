# INSNavGeod block 
# Block_Number 4226

std_msgs/Header header

# SBF block header including time header
BlockHeader block_header

uint8       gnss_mode
uint8       error
uint16      info
uint16      gnss_age
float64     latitude
float64     longitude
float64     height
float32     undulation
uint16      accuracy
uint16      latency
uint8       datum
#uint8       reserved
uint16      sb_list

# INSNavGeodPosStdDev sub-block definition:
# If the Position StdDev field is 1 then this sub block is available:
float32     latitude_std_dev
float32     longitude_std_dev
float32     height_std_dev

# INSNavGeodPosCov sub-block definition:
# If the Position Cov field is 1 then this sub block is available:
float32     latitude_longitude_cov
float32     latitude_height_cov
float32     longitude_height_cov

# INSNavGeodAtt sub-block definition:
# If the Attitude field is 1 then this sub block is available:
float32     heading
float32     pitch
float32     roll

# INSNavGeodAttStdDev sub-block definition:
# If the Attitude StdDev field is 1 then this sub block is available:
float32     heading_std_dev
float32     pitch_std_dev
float32     roll_std_dev

# INSNavGeodAttCov sub-block definition:
# If the Attitude Cov field is 1 then this sub block is available:
float32     heading_pitch_cov
float32     heading_roll_cov
float32     pitch_roll_cov

# INSNavGeodVel sub-block definition:
# If the Velocity field is 1 then this sub block is available:
float32     ve
float32     vn
float32     vu

# INSNavGeodVelStdDev sub-block definition:
# If the Velocity StdDev field is 1 then this sub block is available:
float32     ve_std_dev
float32     vn_std_dev
float32     vu_std_dev

# INSNavGeodVelCov sub-block definition:
# If the Velocity Cov field is 1 then this sub block is available:
float32     ve_vn_cov
float32     ve_vu_cov
float32     vn_vu_cov   
