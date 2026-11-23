# D题统一设备协议v1.7中的可选GCS_STATUS心跳在ROS 2内的完整镜像。
uint8 GCS_STARTING=0
uint8 GCS_READY=1
uint8 GCS_DISPLAYING=2
uint8 GCS_ERROR=240
uint8 FLAG_DISPLAY_READY=1
uint8 FLAG_RECORDING=2

std_msgs/Header header
bool communication_ok
uint8 sequence
uint16 session_id
uint8 gcs_state
uint8 mission_phase_echo
uint8 flags
