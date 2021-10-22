# ExtEventINSNavGeod block 
# Block_Number 4230

Header      header

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
uint8       datum
uint16      sb_list

# ExtEventINSNavGeodPosStdDev sub-block definition:
# If the Position StdDev field is 1 then this sub block is available:
float32     latitude_std_dev
float32     longitude_std_dev
float32     height_std_dev

# ExtEventINSNavGeodAtt sub-block definition:
# If the Attitude field is 1 then this sub block is available:
float32     heading
float32     pitch
float32     roll

# ExtEventINSNavGeodAttStdDev sub-block definition:
# If the Attitude StdDev field is 1 then this sub block is available:
float32     heading_std_dev
float32     pitch_std_dev
float32     roll_std_dev

# ExtEventINSNavGeodVel sub-block definition:
# If the Velocity field is 1 then this sub block is available:
float32     ve
float32     vn
float32     vu

# ExtEventINSNavGeodVelStdDev sub-block definition:
# If the Velocity StdDev field is 1 then this sub block is available:
float32     ve_std_dev
float32     vn_std_dev
float32     vu_std_dev