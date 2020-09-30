# PVTCartesian block
# ROS message header
Header       header

# SBF block header including time header
BlockHeader  Block_Header   

uint8        Mode
uint8        Error        
float64      X           
float64      Y            
float64      Z     
float32      Undulation
float32      Vx         
float32      Vy           
float32      Vz 
float32      COG   
float64      RxClkBias 
float32      RxClkDrift  
uint8        TimeSystem 
uint8        Datum
uint8        NrSV 
uint8        WACorrInfo
uint16       ReferenceID
uint16       MeanCorrAge
uint32       SignalInfo
uint8        AlertFlag
uint8        NrBases
uint16       PPPInfo
uint16       Latency
uint16       HAccuracy
uint16       VAccuracy
uint8        Misc
