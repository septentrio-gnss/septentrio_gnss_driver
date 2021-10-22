# ExtEventINSNavCart block 
# Block_Number 4229

Header      header

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
uint8       datum
uint16      sb_list

# ExtEventINSNavCartPosStdDev sub-block definition:
# If the Position StdDev field is 1 then this sub block is available:
float32     x_std_dev
float32     y_std_dev
float32     z_std_dev

# ExtEventINSNavCartAtt sub-block definition:
# If the Attitude field is 1 then this sub block is available:
float32     heading
float32     pitch
float32     roll

# ExtEventINSNavCartAttStdDev sub-block definition:
# If the Attitude StdDev field is 1 then this sub block is available:
float32     heading_std_dev
float32     pitch_std_dev
float32     roll_std_dev

# ExtEventINSNavCartVel sub-block definition:
# If the Velocity field is 1 then this sub block is available:
float32     vx
float32     vy
float32     vz

# ExtEventINSNavCartVelStdDev sub-block definition:
# If the Velocity StdDev field is 1 then this sub block is available:
float32     vx_std_dev
float32     vy_std_dev
float32     vz_std_dev
