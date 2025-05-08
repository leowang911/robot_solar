# msg/controlData.msg
Header header
int32 distance       # m
uint16 target_yaw    # degree
uint16 roller_speed    #m/s
uint16 yaw           #degree
uint8 robot_state        #status 



#robot_state
#MANUAL = 0
#idle = 1
#AUTORUNNING = 2
#UNLOADING = 3
#LOADING = 4
#ENMERGENCYSTOP = 5
#INIT = 6
#AUTOCORNER = 7
#AUTOWASHING = 8