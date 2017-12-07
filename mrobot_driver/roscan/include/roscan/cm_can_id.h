#ifndef CM_CAN_ID_H
#define CM_CAN_ID_H

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>

#define     POWER_BOARD_NODE_ID     0x50
#define     IPC_NODE_ID             0x01

#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
typedef union {
    struct {
        uint32_t Reserve     : 3;
        uint32_t SrcMacID    : 8;
        uint32_t DstMacID    : 8;
        uint32_t Ack         : 1;
        uint32_t FuncID      : 4;
        uint32_t SourceID    : 8;
    } can_id_stru;
    uint32_t can_id;
} can_id_union;

typedef union
{
	struct
	{
		uint8_t Data[7];
        uint8_t SegPolo : 2;
        uint8_t SegNum  : 6;
	} can_data_stru;
	uint8_t CanData[8];
} can_data_union;
#else
typedef union {
    struct {
        uint32_t SourceID    : 8;
        uint32_t FuncID      : 4;
        uint32_t Ack         : 1;
        uint32_t DstMacID    : 8;
        uint32_t SrcMacID    : 8;
        uint32_t Reserve     : 3;
    } can_id_stru;
    uint32_t can_id;
} can_id_union;

typedef union
{
	struct
	{
        uint8_t SegNum  : 6;
        uint8_t SegPolo : 2;
		uint8_t Data[7];
	} can_data_stru;
	uint8_t CanData[8];
} can_data_union;
#endif //#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__

#endif //#ifndef CM_CAN_ID_H

