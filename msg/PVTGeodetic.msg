# PVTGeodetic block
# ROS message header
Header       ROS_Header

# SBF block header including time header
BlockHeader  Block_Header   

uint8        Mode
uint8        Error        
float64      Latitude          
float64      Longitude     
float64      Height    
float32      Undulation
float32      Vn        
float32      Ve           
float32      Vu
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