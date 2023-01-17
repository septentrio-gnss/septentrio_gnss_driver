# AttCovEuler block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header   

uint8        error   
float32      cov_headhead   # deg^2
float32      cov_pitchpitch # deg^2
float32      cov_rollroll   # deg^2 
float32      cov_headpitch  # deg^2
float32      cov_headroll   # deg^2
float32      cov_pitchroll  # deg^2
