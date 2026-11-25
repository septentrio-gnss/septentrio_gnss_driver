# PVTGeodetic block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header   

uint8        mode
# Type of PVT solution in bits 0-3 of mode. Bits 4-5 are reserved and bits 6-7
# carry independent flags, so mode must be masked with MODE_TYPE_MASK before it
# is compared against these values. 9, 11 and 12 are reserved by the SBF spec.
#------------------------------------------------------------------------------
uint8 MODE_TYPE_MASK             = 15
uint8 MODE_NO_PVT                = 0
uint8 MODE_STANDALONE            = 1
uint8 MODE_DGNSS                 = 2
uint8 MODE_FIXED_LOCATION        = 3
uint8 MODE_RTK_FIXED             = 4
uint8 MODE_RTK_FLOAT             = 5
uint8 MODE_SBAS                  = 6
uint8 MODE_MOVING_BASE_RTK_FIXED = 7
uint8 MODE_MOVING_BASE_RTK_FLOAT = 8
uint8 MODE_PPP                   = 10
# Independent flags. Bit 6 is set if the command setPVTMode, Static, , auto was
# entered and the receiver is still in the process of determining its fixed
# position. Bit 7 is the 2D/3D flag, set in 2D mode (height assumed constant
# and not computed).
uint8 MODE_FLAG_BASE_AUTO        = 64
uint8 MODE_FLAG_2D               = 128
#------------------------------------------------------------------------------
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