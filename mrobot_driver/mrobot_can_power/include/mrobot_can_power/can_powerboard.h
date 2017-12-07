#ifndef   CAN_POWERBOARD_H
#define   CAN_POWERBOARD_H

#include <stdio.h>


#define         SOURCE_ID_FW_VERSION            0x01

#define         SOURCE_ID_SYSTEM_STATUS         0x80
#define         SOURCE_ID_LEDS                  0x81
#define         SOURCE_ID_BATT_VOLTAGE          0x82
#define         SOURCE_ID_BATT_CURRENT          0x83
#define         SOURCE_ID_BATT_PERCENTAGE       0x84
#define         SOURCE_ID_NV                    0x85
#define         SOURCE_ID_PC                    0x86
#define         SOURCE_ID_PRINTER               0x87
#define         SOURCE_ID_ICCARD                0x88        
#define         SOURCE_ID_5V_RES                0x89
#define         SOURCE_ID_12V_RES               0x8A        
#define         SOURCE_ID_24V_RES               0x8B
#define         SOURCE_ID_5V_DCDC               0x8C
#define         SOURCE_ID_12V_DCDC              0x8D
#define         SOURCE_ID_24V_DCDC              0x8E

#define         FUNC_ID_WRITE					0x01
#define         FUNC_ID_READ					0x02
#define			FUNC_ID_TRIGER					0x03
#define			FUNC_ID_RESET					0x06
#define			FUNC_ID_ERROR					0x0F

#define         ACK_NEED                    1
#define         ACK_NOT_NEED                0

#define         SET_RESEND_FLAG             1
#define         SET_END_FLAG                1

typedef enum {
    DEVICE_CONTROL_MODE_READ    = 0x00,
    DEVICE_CONTROL_MODE_OPEN    = 0x01,
    DEVICE_CONTROL_MODE_CLOSE   = 0x02
} device_contorl_t;

typedef enum {
    DEVICE_TYPE_NV          = 0x00,
    DEVICE_TYPE_PC          = 0x01,
    DEVICE_TYPE_PRINTER     = 0x02,
    DEVICE_TYPE_ICCARD      = 0x03,
    DEVICE_TYPE_5V_RES      = 0x04,
    DEVICE_TYPE_12V_RES     = 0x05,
    DEVICE_TYPE_24V_RES     = 0x06,
    DEVICE_TYPE_5V_DCDC     = 0x07,
    DEVICE_TYPE_12V_DCDC    = 0x08,
    DEVICE_TYPE_24V_DCDC    = 0x09,
    DEVICE_TYPE_MAX         = 0x0A
} device_type_t;

#endif

