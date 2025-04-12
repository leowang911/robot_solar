# msg/controlData.msg
Header header
uint32 distance       # m
uint16 target_yaw    # degree
uint16 roller_speed    #m/s
uint16 yaw           #degree
uint8 robot_state        #status 



#robot_state
#MANUAL = 0
#AUTORUNNING = 1
#UNLOADING = 2
#LOADING = 3
#ENMERGENCYSTOP = 4
#INIT = 5
#AUTOCORNER = 6
#AUTOWASHING = 7