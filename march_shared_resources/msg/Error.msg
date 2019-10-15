# Readable message that describes the error
Header header
string error_message

# The type of the error, the type is used to determine how to handle the error
uint8 type
uint8 FATAL             = 0     # The error indicates a serious malfunctioning of the system this is unrecoverable
uint8 NON_FATAL         = 1     # The error indicates a malfunctioning of the system this can be recoverable
