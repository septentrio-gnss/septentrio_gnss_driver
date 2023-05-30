# MeasEpochChannelType2 block
   
uint8  type
uint8  lock_time          # s
uint8  cn0                # 0.25 dB-Hz
uint8  offsets_msb        # 65.536 m or 65.536 Hz
int8   carrier_msb        # 65.536 cycles
uint8  obs_info
uint16 code_offset_lsb    # 0.001 m
uint16 carrier_lsb        # 0.001 cycles
uint16 doppler_offset_lsb # 0.0001 Hz