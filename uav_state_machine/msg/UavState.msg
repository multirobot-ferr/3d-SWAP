# Define all possible states as constants here:
uint8 REPOSE=0
uint8 TAKINGOFF=1
uint8 HOVER=2
uint8 SEARCHING=3
uint8 CATCHING=4
uint8 LANDING=5
uint8 GOTO_DEPLOY=6
uint8 GOTO_CATCH=7
uint8 RETRY_CATCH=8
uint8 ERROR=9

# And here message itself:
uint8 uav_id
uint8 state
std_msgs/String 	state_str
std_msgs/String 	state_msg

