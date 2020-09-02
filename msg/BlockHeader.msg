# Blockheader including time header for all ROS messages that publish SBF blocks

uint8     SYNC1
uint8     SYNC2
uint16    CRC
uint16    ID
uint16    Length

uint32    TOW
uint16    WNc     