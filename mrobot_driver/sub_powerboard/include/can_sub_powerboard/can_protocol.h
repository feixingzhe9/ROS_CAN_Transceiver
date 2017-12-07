#ifndef _CAN_PROTOCOL_H
#define _CAN_PROTOCOL_H

typedef union
{
	struct
	{
		uint32_t SourceID  : 8;
		uint32_t FUNC_ID   : 4;
		uint32_t ACK       : 1;
		uint32_t DestMACID : 8;
		uint32_t SrcMACID  : 8;
		uint32_t res       : 3;
	}CanID_Struct;
	uint32_t  CANx_ID;
}CAN_ID_UNION;

typedef union
{
	struct
	{
        uint8_t SegNum  : 6;
        uint8_t SegPolo : 2;
		uint8_t Data[7];
	}CanData_Struct;
	uint8_t CanData[8];
}CAN_DATA_UNION;
 

#endif
