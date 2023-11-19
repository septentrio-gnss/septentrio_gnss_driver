# PVTGeodetic block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header   

uint8        mode
uint8        error        
float64      latitude      # rad    
float64      longitude     # rad
float64      height        # m (ellipsoidal)
float32      undulation    # m
float32      vn            # m/s
float32      ve            # m/s
float32      vu            # m/s
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