# AIMPlusStatus message
# ROS message header
std_msgs/Header header

uint32 tow # ms since week start
uint16 wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     

uint8 interference
#-------------------------------
uint8 SPECTRUM_CLEAN         = 0
uint8 INTERFERENCE_MITIGATED = 1
uint8 INTERFERENCE_PRESENT   = 2
#-------------------------------

uint8 spoofing
#--------------------------------------------------------
uint8 NONE_DETECTED                                   = 0
uint8 SPOOFING_DETECTED_BY_OSNMA                      = 1
uint8 SPOOFING_DETECTED_BY_AUTHENTCITY_TEST           = 2
uint8 SPOOFING_DETECTED_BY_OSNMA_AND_AUTHENTCITY_TEST = 3
#--------------------------------------------------------

bool  osnma_authenticating
uint8 galileo_authentic
uint8 galileo_spoofed
uint8 gps_authentic
uint8 gps_spoofed