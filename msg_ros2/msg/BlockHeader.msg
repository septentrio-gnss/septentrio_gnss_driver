# Blockheader including time header for all ROS messages that publish SBF blocks
# This is the ROS 2 version of the message. The ROS 1 version is at
# msg/BlockHeader.msg .

uint8     sync_1 36 # 0x24
uint8     sync_2 64 # 0x40
uint16    crc
uint16    id
uint8     revision
uint16    length

uint32    tow 4294967295 # ms since week start
uint16    wnc 65535 # weeks since Jan 06, 1980 at 00:00:00 UTC     
