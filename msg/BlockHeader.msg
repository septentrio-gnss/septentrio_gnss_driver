# Blockheader including time header for all ROS messages that publish SBF blocks

uint8     sync_1 # 0x24
uint8     sync_2 # 0x40
uint16    crc
uint16    id
uint8     revision
uint16    length

uint32    tow # ms since week start
uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     