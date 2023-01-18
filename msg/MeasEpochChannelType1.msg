# MeasEpochChannelType1 block
   
uint8  rx_channel
uint8  type
uint8  sv_id
uint8  misc        # 4294967.296 m
uint32 code_lsb    # 0.001 m
int32  doppler     # 0.0001 Hz
uint16 carrier_lsb # 0.001 cycles
int8   carrier_msb # 65.536 cycles
uint8  cn0         # 0.25 dB-Hz
uint16 lock_time
uint8  obs_info
uint8  n2

MeasEpochChannelType2[] type2