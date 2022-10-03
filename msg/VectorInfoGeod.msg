# VectorInfoGeod block
   
uint8   nr_sv
uint8   error
uint8   mode
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