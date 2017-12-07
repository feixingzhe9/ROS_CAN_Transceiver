#ifndef CM_CAN_ID_H
#define CM_CAN_ID_H

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>

#define     POWER_BOARD_NODE_ID     0x08
#define     IPC_NODE_ID             0x00

typedef enum {
    SrcMacIdBit         = 21,
    DstMacIdBit         = 13,
    AckBit              = 12,
    FuncIdBit           = 8,
    SourceIdBit         = 0,
} CM_CAN_ID_BIT;

typedef enum {
    SrcMacIdMask        = 0x1fe00000,
    DstMacIdMask        = 0x001fe000,
    AckMask             = 0x00001000,
    FuncIdMask          = 0x00000f00,
    SourceIdMask        = 0x000000ff,
} CM_CAN_ID_MASK;

typedef struct {
    uint8_t         SrcMacId;
    uint8_t         DstMacId;
    uint8_t         Ack;
    uint8_t         FuncId;
    uint8_t         SourceId;
} cm_can_id_t;

class CM_CAN_ID
{
    public:
        cm_can_id_t cmCanIdParse(uint32_t id);
        uint32_t cmCanIdBuild(cm_can_id_t* CanID);
};

#endif

