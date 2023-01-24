# AttEuler block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header   

uint8        nr_sv
uint8        error        
uint16       mode
float32      heading     # deg
float32      pitch       # deg
float32      roll        # deg
float32      pitch_dot   # deg/s
float32      roll_dot    # deg/s
float32      heading_dot # deg/s
