# PVTCartesian block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header   

uint8        mode
uint8        error        
float64      x           
float64      y            
float64      z     
float32      undulation
float32      vx         
float32      vy           
float32      vz 
float32      cog   
float64      rx_clk_bias 
float32      rx_clk_drift  
uint8        time_system 
uint8        datum
uint8        nr_sv 
uint8        wa_corr_info
uint16       reference_id
uint16       mean_corr_age
uint32       signal_info
uint8        alert_flag
uint8        nr_bases
uint16       ppp_info
uint16       latency
uint16       h_accuracy
uint16       v_accuracy
uint8        misc
