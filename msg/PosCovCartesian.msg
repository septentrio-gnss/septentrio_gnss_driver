# PVTGeodetic block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header   

uint8        mode
uint8        error        
float32      cov_xx
float32      cov_yy
float32      cov_zz
float32      cov_bb
float32      cov_xy
float32      cov_xz
float32      cov_xb
float32      cov_yz
float32      cov_yb
float32      cov_zb