# The type of instruction.
int16 type

# All possible instruction types.
int16 GAIT = 0
int16 CONTINUE = 1
int16 PAUSE = 2
int16 STOP = 3

# If the type is a GAIT this is the name of the gait.
string gait_name
