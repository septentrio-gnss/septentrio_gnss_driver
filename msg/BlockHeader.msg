# Blockheader including time header for all ROS messages that publish SBF blocks
# This is the ROS 1 version of the message file. The ROS 2 version is
# msg_ros2/msg/BlockHeader.msg .

uint8     sync_1
uint8     sync_2
uint16    crc
uint16    id
uint8     revision
uint16    length

uint32    tow # ms since week start
uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     
