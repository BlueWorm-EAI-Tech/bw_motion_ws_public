# Represents a generic event in the state machine system

# --- Constants for event_type for clear, programmatic usage ---
uint8 UNDEFINED = 0
uint8 JOY_READY = 1
uint8 VR_READY = 2
uint8 LEFT_POSE_RESET = 3
uint8 RIGHT_POSE_RESET = 4
uint8 LEFT_START_TELEOP = 5
uint8 RIGHT_START_TELEOP = 6
uint8 LEFT_STOP_TELEOP = 7
uint8 RIGHT_STOP_TELEOP = 8
uint8 SWITCH_MODE = 10

# Type of the event, use one of the constants defined above
uint8 event_type

# Optional human-readable description of the event
string description