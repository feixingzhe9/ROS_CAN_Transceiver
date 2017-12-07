#ifndef   CAN_POWERBOARD_H
#define   CAN_POWERBOARD_H

#include <stdio.h>


#define         FUNC_ID_LEDS_CONTROL        0x20
#define         FUNC_ID_S_SYS_V_BAT         0x21
#define         FUNC_ID_MODULE_STATE        0x22
#define         FUNC_ID_FAULT_BIT           0x23
#define         FUNC_ID_MODULE_CONTROL      0x24
#define         FUNC_ID_IRLED_CONTROL       0x25
#define         FUNC_ID_FANS_CONTROL        0x26
#define         FUNC_ID_TEST_CURRENT        0x27

#define         FUNC_ID_READ_ERR_CURRENT    0x29
#define         FUNC_ID_VERSION_INFO        0x2A
#define         FUNC_ID_FW_UPGRADE          

#define         ACK_NEED                    1
#define         ACK_NOT_NEED                0

#define         SET_RESEND_FLAG             1
#define         SET_END_FLAG                1

#endif

