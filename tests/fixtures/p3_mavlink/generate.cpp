// Offline synthetic codec fixtures. Never sends a packet or starts PX4/QGC.
#include <development/mavlink.h>
#include <fstream>
#include <string>

static void save(const std::string &root,const char *name,mavlink_message_t &message) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN]{};
    const auto size=mavlink_msg_to_send_buffer(buffer,&message);
    std::ofstream(root+"/"+name,std::ios::binary).write(reinterpret_cast<char *>(buffer),size);
}
int main(int argc,char **argv) {
    if(argc!=2)return 2;
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(255,190,&message,MAV_TYPE_GCS,MAV_AUTOPILOT_INVALID,0,0,MAV_STATE_ACTIVE);
    save(argv[1],"heartbeat-v2.bin",message);
    mavlink_msg_command_long_pack(255,190,&message,72,1,MAV_CMD_RUN_PREARM_CHECKS,0,0,0,0,0,0,0,0);
    save(argv[1],"command401-v2.bin",message);
    mavlink_get_channel_status(MAVLINK_COMM_0)->flags|=MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    mavlink_msg_command_long_pack(255,190,&message,72,1,MAV_CMD_REQUEST_MESSAGE,0,MAVLINK_MSG_ID_AUTOPILOT_VERSION,0,0,0,0,0,0);
    save(argv[1],"request512-v1.bin",message);
    mavlink_get_channel_status(MAVLINK_COMM_0)->flags&=~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    mavlink_msg_command_int_pack(255,190,&message,72,1,MAV_FRAME_LOCAL_NED,MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,10,20,3);
    save(argv[1],"waypoint16-v2.bin",message);
    mavlink_msg_command_cancel_pack(255,190,&message,72,1,MAV_CMD_RUN_PREARM_CHECKS);
    save(argv[1],"cancel401-v2.bin",message);
}
