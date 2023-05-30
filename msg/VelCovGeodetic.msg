# VelCovGeodetic block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header   

uint8        mode
uint8        error        
float32      cov_vnvn # m^2/s^2
float32      cov_veve # m^2/s^2
float32      cov_vuvu # m^2/s^2
float32      cov_dtdt # m^2/s^2
float32      cov_vnve # m^2/s^2
float32      cov_vnvu # m^2/s^2
float32      cov_vndt # m^2/s^2
float32      cov_vevu # m^2/s^2
float32      cov_vedt # m^2/s^2
float32      cov_vudt # m^2/s^2