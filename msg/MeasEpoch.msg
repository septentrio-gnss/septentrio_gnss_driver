# MeasEpoch block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header   

uint8 n
uint8 sb1_length
uint8 sb2_length      
uint8 common_flags
uint8 cum_clk_jumps

MeasEpochChannelType1[] type1