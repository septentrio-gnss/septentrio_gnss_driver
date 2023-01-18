# PVTGeodetic block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header   

uint8        mode
uint8        error        
float32      cov_xx # m^2
float32      cov_yy # m^2
float32      cov_zz # m^2
float32      cov_bb # m^2
float32      cov_xy # m^2
float32      cov_xz # m^2
float32      cov_xb # m^2
float32      cov_yz # m^2
float32      cov_yb # m^2
float32      cov_zb # m^2