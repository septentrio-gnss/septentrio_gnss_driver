# VectorInfoGeod block
   
uint8   nr_sv
uint8   error
uint8   mode
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
uint8   misc
float64 delta_east   # m
float64 delta_north  # m
float64 delta_up     # m
float32 delta_ve     # m
float32 delta_vn     # m
float32 delta_vu     # m
uint16  azimuth      # 0.01 deg
int16   elevation    # 0.01 deg
uint16  reference_id
uint16  corr_age     # 0.01 s
uint32  signal_info