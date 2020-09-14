# AttEuler block
# ROS message header
Header       ROS_Header

# SBF block header including time header
BlockHeader  Block_Header   

uint8        NrSV
uint8        Error        
uint16       Mode
float32      Heading
float32      Pitch
float32      Roll
float32      PitchDot
float32      RollDot
float32      HeadingDot
