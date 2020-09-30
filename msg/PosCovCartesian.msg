# PVTGeodetic block
# ROS message header
Header       header

# SBF block header including time header
BlockHeader  Block_Header   

uint8        Mode
uint8        Error        
float32      Cov_xx
float32      Cov_yy
float32      Cov_zz
float32      Cov_bb
float32      Cov_xy
float32      Cov_xz
float32      Cov_xb
float32      Cov_yz
float32      Cov_yb
float32      Cov_zb