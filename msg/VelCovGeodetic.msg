# VelCovGeodetic block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header   

uint8        mode
uint8        error        
float32      cov_vnvn
float32      cov_veve
float32      cov_vuvu
float32      cov_dtdt
float32      cov_vnve
float32      cov_vnvu
float32      cov_vndt
float32      cov_vevu
float32      cov_vedt
float32      cov_vudt