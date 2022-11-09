# VelCovCartesian block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header   

uint8        mode
uint8        error        
float32      cov_vxvx
float32      cov_vyvy
float32      cov_vzvz
float32      cov_dtdt
float32      cov_vxvy
float32      cov_vxvz
float32      cov_vxdt
float32      cov_vyvz
float32      cov_vydt
float32      cov_vzdt