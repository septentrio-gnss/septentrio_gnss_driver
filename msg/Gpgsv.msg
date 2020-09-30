# From https://www.gpsinformation.org/dale/nmea.htm#GSA: GSV shows data about the satellites 
# that the unit might be able to find based on its viewing mask and almanac data. It also shows current ability 
# to track this data. Note that one GSV sentence only can provide data for up to 4 satellites and thus there may 
# need to be 3 sentences for the full information. It is reasonable for the GSV sentence to contain more satellites 
# than GGA might indicate since GSV may include satellites that are not used as part of the solution. It is not a 
# requirment that the GSV sentences all appear in sequence.

Header header

string message_id

# Number of messages for full data
uint8 n_msgs

# This message's number. The first message has number 1, last has number n_msgs.
uint8 msg_number

# Number of satellites currently in view
uint8 n_satellites

# Up to 4 satellites can be described in each message
GpgsvSatellite[] satellites