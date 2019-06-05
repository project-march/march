# The type of instruction.
int16 type

# All possible instruction types.
int16 STOP = 0
int16 CONTINUE = 1
int16 GAIT = 2
int16 PAUSE = 3

# If the type is a GAIT this is the name of the gait.
string gait_name
