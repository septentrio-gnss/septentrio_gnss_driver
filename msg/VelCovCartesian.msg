# VelCovCartesian block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header   

uint8        mode
uint8        error        
float32      cov_vxvx # m^2/s^2
float32      cov_vyvy # m^2/s^2
float32      cov_vzvz # m^2/s^2
float32      cov_dtdt # m^2/s^2
float32      cov_vxvy # m^2/s^2
float32      cov_vxvz # m^2/s^2
float32      cov_vxdt # m^2/s^2
float32      cov_vyvz # m^2/s^2
float32      cov_vydt # m^2/s^2
float32      cov_vzdt # m^2/s^2