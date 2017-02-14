# An alert message, that notifies the user something is wrong
# inside the robot or around it.

# the header, useful for the timestamp
Header header

# the priority, one of the constants defined below
uint8 priority
uint8 PRIORITY_DEBUG= 0
uint8 PRIORITY_INFO = 1
uint8 PRIORITY_WARN = 2
uint8 PRIORITY_SEVERE = 3

# the message explaining the error
string message

# an optional sentence to be said (NL syntax, for instance "|en:Careful!")
string etts_sentence

# an optional image showing what happened
sensor_msgs/Image image
