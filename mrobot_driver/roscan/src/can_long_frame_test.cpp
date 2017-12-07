#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <fcntl.h>
#include <mrobot_driver_msgs/vci_can.h>
#include <roscan/cm_can_id.h>
#include <roscan/can_long_frame.h>
#include <can_interface/CanMsg.h>

#if 1
bool can_send_handle(can::CanMsg* Msg){;}
int main(int argc, char **argv)
{
    mrobot_driver_msgs::vci_can can_msg;
    mrobot_driver_msgs::vci_can can_msg_send;
    mrobot_driver_msgs::vci_can canmsg[3];

    can::CanMsg  CMsg[2];
#if 0
    CMsg[0].setID(0x1234567);
    CMsg[0].setLength(8);
    CMsg[0].set(0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07);
#else
    CMsg[0].setID(0x1234567);
    CMsg[0].setLength(8);
    CMsg[0].set(0x40,0x01,0x02,0x03,0x04,0x05,0x06,0x07);
    CMsg[1].setID(0x1234567);
    CMsg[1].setLength(8);
    CMsg[1].set(0xc1,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e);
    canmsg[0].ID = 0x1234556;
    canmsg[0].DataLen = 8;
    canmsg[0].Data = {0x40,0x01,0x02,0x03,0x04,0x05,0x06,0x07};
    canmsg[1].ID = 0x1234556;
    canmsg[1].DataLen = 8;
    canmsg[1].Data = {0xc1,0x01,0x02,0x03,0x04,0x05,0x06,0x07};
#endif
    can_msg_send.ID = 0x11335577;
    can_msg_send.DataLen = 16;
    can_msg_send.Data = {0x01,0x02,0x03,0x07,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,0xff};
    can_long_frame long_frame;
    can_id_union can_id_u;
    while (1)
    {
        can_msg = long_frame.frame_construct(&canmsg[0]);//&CMsg[0]);
        can_msg = long_frame.frame_construct(&canmsg[1]);//&CMsg[1]);
        can_id_u.can_id = can_msg.ID;
        std::cout << "CAN.SrcMacID:" << std::hex << can_id_u.can_id_stru.SrcMacID << std::endl;
        switch( can_id_u.can_id_stru.SrcMacID )
        {
            case POWER_BOARD_NODE_ID:
                break;
            default:
                std::cout << "CAN.ID:" << std::hex << can_msg.ID << std::endl;
                break;
        }
        long_frame.frame_parser(&can_msg_send, can_send_handle);
        sleep(1);
    }

    return 0;
}
#endif

