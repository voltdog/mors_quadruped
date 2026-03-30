# robot mode:
# 0 - locomotion mode 
# 1 - leg control mode
# 2 - body mode
# 3 - joint control
# 4 - do nothing 
# uint8 robot_mode

# number of chosen action:
# 0 - no action
# 1 - standup
# 2 - lay down
# 3 - give paw
# actions will be executing only if robot_mode=3
# uint8 action_num

# type of body adaptation to environment
# 0 - height and pitch adaptation
# 1 - only height adaptation
# 2 - no adaptation
# uint8 adaptation_type

# gait parameters
float32 stride_height
float32 t_sw
float32 t_st
float32[4] gait_offsets
bool standing
