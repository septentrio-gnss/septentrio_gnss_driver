# PVTGeodetic block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header   

uint8        mode
uint8        error        
float32      cov_latlat # m^2
float32      cov_lonlon # m^2
float32      cov_hgthgt # m^2
float32      cov_bb     # m^2
float32      cov_latlon # m^2
float32      cov_lathgt # m^2
float32      cov_latb   # m^2
float32      cov_lonhgt # m^2
float32      cov_lonb   # m^2
float32      cov_hb     # m^2