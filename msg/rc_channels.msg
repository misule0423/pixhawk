uint8 RC_CHANNELS_FUNCTION_FORM = 16                         # test
int32 RC_CHANNELS_FUNCTION_MAX=19
uint8 RC_CHANNELS_FUNCTION_THROTTLE=0
uint8 RC_CHANNELS_FUNCTION_ROLL=1
uint8 RC_CHANNELS_FUNCTION_PITCH=2
uint8 RC_CHANNELS_FUNCTION_YAW=3
uint8 RC_CHANNELS_FUNCTION_MODE=4
uint8 RC_CHANNELS_FUNCTION_RETURN=5
uint8 RC_CHANNELS_FUNCTION_POSCTL=6
uint8 RC_CHANNELS_FUNCTION_LOITER=7
uint8 RC_CHANNELS_FUNCTION_OFFBOARD=8
uint8 RC_CHANNELS_FUNCTION_ACRO=9
uint8 RC_CHANNELS_FUNCTION_FLAPS=10
uint8 RC_CHANNELS_FUNCTION_AUX_1=11
uint8 RC_CHANNELS_FUNCTION_AUX_2=12
uint8 RC_CHANNELS_FUNCTION_AUX_3=13
uint8 RC_CHANNELS_FUNCTION_AUX_4=14
uint8 RC_CHANNELS_FUNCTION_AUX_5=15
uint8 RC_CHANNELS_FUNCTION_PARAM_1=16
uint8 RC_CHANNELS_FUNCTION_PARAM_2=17
uint8 RC_CHANNELS_FUNCTION_PARAM_3_5=18
uint64 timestamp						# Timestamp in microseconds since boot time
uint64 timestamp_last_valid					# Timestamp of last valid RC signal
float32[19] channels						# Scaled to -1..1 (throttle: 0..1)
uint8 channel_count						# Number of valid channels
int8[19] function						# Functions mapping
uint8 rssi							# Receive signal strength index
bool signal_lost						# Control signal lost, should be checked together with topic timeout
