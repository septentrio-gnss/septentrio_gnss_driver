# GALAuthStatus block 
# Block_Number 4245

std_msgs/Header header

# SBF block header including time header
BlockHeader block_header

uint16  osnma_status
float32 trusted_time_delta # s
uint64  gal_active_mask
uint64  gal_authentic_mask
uint64  gps_active_mask
uint64  gps_authentic_mask