# VectorInfoCart block
   
uint8   nr_sv
uint8   error
uint8   mode
uint8   misc
float64 delta_x      # m
float64 delta_y      # m
float64 delta_z      # m
float32 delta_vx     # m
float32 delta_vy     # m
float32 delta_vz     # m
uint16  azimuth      # 0.01 deg
int16   elevation    # 0.01 deg
uint16  reference_id
uint16  corr_age     # 0.01 s
uint32  signal_info