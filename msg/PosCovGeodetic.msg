# PVTGeodetic block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header   

uint8        mode
uint8        error        
float32      cov_latlat
float32      cov_lonlon
float32      cov_hgthgt
float32      cov_bb
float32      cov_latlon
float32      cov_lathgt
float32      cov_latb
float32      cov_lonhgt
float32      cov_lonb
float32      cov_hb