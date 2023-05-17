# ReceiverTime block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header

int8  utc_year    # year
int8  utc_month   # month
int8  utc_day     # day
int8  utc_hour    # hour
int8  utc_min     # minute
int8  utc_second # s
int8  delta_ls   # s
uint8 sync_level