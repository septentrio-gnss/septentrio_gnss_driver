# PVTGeodetic block
# ROS message header
Header       ROS_Header

# SBF block header including time header
BlockHeader  Block_Header   

uint8        Mode
uint8        Error        
float32      Cov_latlat
float32      Cov_lonlon
float32      Cov_hgthgt
float32      Cov_bb
float32      Cov_latlon
float32      Cov_lathgt
float32      Cov_latb
float32      Cov_lonhgt
float32      Cov_lonb
float32      Cov_hb