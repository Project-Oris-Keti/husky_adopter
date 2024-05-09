# Diagnostics bitfield https://docs.google.com/spreadsheets/d/1GEfCuSiSrR_jbs1qqKKAbcMzjDvsZ1FzZxI7YDJTqM8/edit#gid=0
uint32 DIAG_CAM0 = 128
uint32 DIAG_MOTOR = 32768
uint32 DIAG_RUN = 131072
uint32 DIAG_PAYLOAD = 262144
uint32 DIAG_STAND = 524288
uint32 DIAG_COLL_AVD = 1048576
uint32 DIAG_IMU = 2097152
uint32 DIAG_CLIFF = 4194304
uint32 DIAG_STAIRS_HEADING = 8388608
uint32 DIAG_LOADING = 16777216
uint32 DIAG_BATT_LOW_ERR = 33554432
uint32 DIAG_BATT_LOW_WARN = 67108864
uint32 DIAG_LOST = 134217728
uint32 DIAG_AUT_PROG = 16384

uint8 estop
uint8 control_mode
uint8 action
uint8 si_units
uint8 vision_mode
uint8 high_step
uint8 blind_stairs
uint8 run
uint8 dock
uint8 recovery
uint8 hill
uint8 sand
uint8 soft_estop
uint8 arm
uint8 leg_lock
uint8 pose_rate
uint8 inverted

# Diagnostics bitfields
uint32 present
uint32 enabled
uint32 health
