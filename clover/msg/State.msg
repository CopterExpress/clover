uint8 MODE_NONE = 0
uint8 MODE_NAVIGATE = 1
uint8 MODE_NAVIGATE_GLOBAL = 2
uint8 MODE_POSITION = 3
uint8 MODE_VELOCITY = 4
uint8 MODE_ATTITUDE = 5
uint8 MODE_RATES = 6

uint8 YAW_MODE_YAW = 0
uint8 YAW_MODE_YAW_RATE = 1
uint8 YAW_MODE_YAW_TOWARDS = 2

# type of offboard control
uint8 mode
uint8 yaw_mode

# targets
float32 x
float32 y
float32 z
float32 speed
float32 lat
float32 lon
float32 vx
float32 vy
float32 vz
float32 roll
float32 pitch
float32 yaw
float32 roll_rate
float32 pitch_rate
float32 yaw_rate
float32 thrust

# frames of reference
string xy_frame_id
string z_frame_id
string yaw_frame_id
