# The type of instruction.
int16 type

# All possible instruction types.
int16 STOP = 0
int16 GAIT = 1
int16 PAUSE = 2
int16 CONTINUE = 3
int16 INPUT_DEVICE_LOST = 4

# If the type is a GAIT this is the name of the gait.
string gait_name
