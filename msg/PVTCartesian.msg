# PVTCartesian block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header   

uint8        mode
uint8        error        
float64      x             # m    
float64      y             # m             
float64      z             # m      
float32      undulation    # m 
float32      vx            # m/s         
float32      vy            # m/s 
float32      vz            # m/s
float32      cog           # deg
float64      rx_clk_bias   # ms
float32      rx_clk_drift  # ppm
uint8        time_system 
uint8        datum
uint8        nr_sv 
uint8        wa_corr_info
uint16       reference_id
uint16       mean_corr_age # 0.01s
uint32       signal_info
uint8        alert_flag
uint8        nr_bases
uint16       ppp_info      # s
uint16       latency       # 0.0001 s
uint16       h_accuracy    # 0.01 m
uint16       v_accuracy    # 0.01 m
uint8        misc
