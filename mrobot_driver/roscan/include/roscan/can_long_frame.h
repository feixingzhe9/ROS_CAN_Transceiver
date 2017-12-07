#ifndef _CAN_LONG_FRAME_H
#define _CAN_LONG_FRAME_H

#include <iostream>
#include <iomanip>
#include <sys/types.h>
#include <can_interface/CanMsg.h>
#include <mrobot_driver_msgs/vci_can.h>

#define CAN_BUF_NO_THIS_ID          0xfe
#define CAN_LONG_FRAME_TIME_OUT     5000/SYSTICK_PERIOD
#define CAN_COMM_TIME_OUT           5000
#define CAN_LONG_BUF_FULL           0xff

#define ONLYONCE       0x00
#define BEGIAN         0x01
#define TRANSING       0x02
#define END            0x03


#define CAN_ONE_FRAME_DATA_LENTH    7
#define CAN_SEG_NUM_MAX             64
#define CAN_LONG_FRAME_LENTH_MAX    (CAN_ONE_FRAME_DATA_LENTH*CAN_SEG_NUM_MAX)
#define CAN_LONG_BUF_NUM    10

class can_long_frame_buf 
{
    public:
    int one_frame_data_length;
    int can_seg_num_max;
    int can_long_frame_length_max;
    int can_long_frame_buf_num;

    can_long_frame_buf ()
    {
        one_frame_data_length = 7;//CAN_ONE_FRAME_DATA_LENTH    
        can_seg_num_max = 64;//CAN_SEG_NUM_MAX
        can_long_frame_length_max = 7*64;//CAN_LONG_FRAME_LENTH_MAX 
        can_long_frame_buf_num = 10;//CAN_LONG_BUF_NUM

        for( int i=0; i < CAN_LONG_BUF_NUM; i++)
        {
            rcv_buf[i] = new can_rcv_buf();
        }
    }

    ~can_long_frame_buf ()
    {
        for( int i=0; i < CAN_LONG_BUF_NUM; i++)
        {
            delete rcv_buf[i];
        }
    }

    class can_rcv_buf
    {
        public:
        uint32_t can_id;
        uint32_t start_time; 
        int used_len;
        uint8_t  data[CAN_LONG_FRAME_LENTH_MAX];   
    };

    can_rcv_buf *rcv_buf[CAN_LONG_BUF_NUM];

    void MemcpyDataToBuf( int index, int pos, const void *data, int data_len )
    {
        memcpy( &rcv_buf[index]->data[pos], data, data_len );
    }

    void MemcpyDataFromBuf( int index, int pos, void *data, int data_len )
    {
        memcpy( data, &rcv_buf[index]->data[pos], data_len);
    }

    void addBufUsedLen( int index, int len )
    {
        rcv_buf[index]->used_len += len;
    }

    int getBufUsedLen( int index )
    {
        return rcv_buf[index]->used_len;
    }

    int getBufDataAt( int index, int pos )
    {
        return rcv_buf[index]->data[pos];
    }

    uint32_t getBufCanId( int index )
    {
        return rcv_buf[index]->can_id;
    }

    void setBufCanId( int index, uint32_t can_id )
    {
        rcv_buf[index]->can_id = can_id;
    }

    void setBufStartTime( int index, uint32_t start_time ) {}

    int GetOneFreeBuf(void)
    {
        for(int i = 0; i < can_long_frame_buf_num; i++)
        {
            if (rcv_buf[i]->used_len == 0)
            {
                return i;
            }
        }
        return CAN_LONG_BUF_FULL;
    }

    void FreeBuf(int index)
    {
        rcv_buf[index]->can_id = 0;
        rcv_buf[index]->used_len = 0;
        memset(&rcv_buf[index]->data[0], 0x00, CAN_LONG_BUF_NUM);
    }

    int GetTheBufById(uint32_t id)
    {
        for(int i = 0; i < can_long_frame_buf_num; i++)
        {
            if (id == rcv_buf[i]->can_id)
            {
                return i;
            }
        }
        return CAN_BUF_NO_THIS_ID;
    }
};

typedef union
{
	struct
	{
        uint8_t SegNum  : 6;
        uint8_t SegPolo : 2;
		uint8_t Data[7];
	} CanData_Struct;
	uint8_t CanData[8];
} CAN_DATA_UNION;

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
	} CanID_Struct;
	uint32_t  CANx_ID;
} CAN_ID_UNION;


class can_long_frame
{
    public:

    typedef bool (*sendhandle_fn)(can::CanMsg*);

    can_long_frame_buf long_frame_buf;

    can_long_frame()
    {
    }

    uint32_t can_comm_start_time;

    mrobot_driver_msgs::vci_can frame_construct( const mrobot_driver_msgs::vci_can::ConstPtr& CMsg )
    {
        mrobot_driver_msgs::vci_can can_msg;

        CAN_ID_UNION id;
        CAN_DATA_UNION rx_buf;
        int buf_index;
        int seg_polo;
        int seg_num;

        for( int i = 0 ; i < CMsg->DataLen; i++ )
        {
            rx_buf.CanData[i] = CMsg->Data[i];
        }

        seg_polo = rx_buf.CanData_Struct.SegPolo;
        seg_num = rx_buf.CanData_Struct.SegNum;

        if (seg_polo == ONLYONCE)
        {
            id.CANx_ID = CMsg->ID;
#if 0
            std::cout << "short frame buf len:" << std::dec << CMsg->DataLen - 1 << std::endl;
            for(int cnt = 1; cnt < CMsg->DataLen; cnt++)
            {
                std::cout << "short frame buf data[" << cnt -1 << "] = 0x" << std::setw(2) << std::setfill('0') << std::hex <<  CMsg->Data[cnt] << std::endl;		
            }
#endif
            can_msg.ID = CMsg->ID;
            can_msg.DataLen = CMsg->DataLen - 1;
            can_msg.Data.resize( CMsg->DataLen - 1);
            memcpy(&can_msg.Data[0], &rx_buf.CanData[1], CMsg->DataLen - 1);  
        }
        else //long frame
        {
            if(seg_polo == BEGIAN)
            {
                buf_index = long_frame_buf.GetTheBufById( CMsg->ID );
                if( buf_index == CAN_BUF_NO_THIS_ID )
                {
                    buf_index = long_frame_buf.GetOneFreeBuf();
                }

                if((buf_index == CAN_LONG_BUF_FULL) || (buf_index >= CAN_LONG_BUF_NUM))
                {
                    std::cout << "LONG FRAME RCV BUF IS FULL! ! ! !" << std::endl;
                    goto exit;
                }
                long_frame_buf.MemcpyDataToBuf(buf_index, 0, rx_buf.CanData_Struct.Data, long_frame_buf.one_frame_data_length);
                long_frame_buf.addBufUsedLen( buf_index, long_frame_buf.one_frame_data_length );
                long_frame_buf.setBufCanId( buf_index, CMsg->ID );
                //can_long_frame_buf->can_rcv_buf[buf_index].start_time = os_get_time();
            }
            else if((seg_polo == TRANSING) || (seg_polo == END))
            {
                buf_index = long_frame_buf.GetTheBufById( CMsg->ID );
                if((buf_index == CAN_BUF_NO_THIS_ID) || (buf_index >= CAN_LONG_BUF_NUM))
                {
                    std::cout << "ERROR ! !" << std::endl;
                    goto exit;
                }
                //can_long_frame_buf->can_rcv_buf[buf_index].start_time = os_get_time();
                if(seg_polo == TRANSING)
                {
                    long_frame_buf.MemcpyDataToBuf(buf_index, seg_num*long_frame_buf.one_frame_data_length, rx_buf.CanData_Struct.Data, long_frame_buf.one_frame_data_length);
                    long_frame_buf.addBufUsedLen( buf_index, long_frame_buf.one_frame_data_length );
                }
                if(seg_polo == END)
                {
                    long_frame_buf.MemcpyDataToBuf(buf_index, seg_num*long_frame_buf.one_frame_data_length, rx_buf.CanData_Struct.Data, CMsg->DataLen - 1);
                    long_frame_buf.addBufUsedLen( buf_index, CMsg->DataLen - 1 ); 
#if 0
                    for(int cnt = 0; cnt < long_frame_buf.getBufUsedLen(buf_index); cnt++)
                    {
                        std::cout << "long buf data[" << std::dec << cnt << "] = 0x" << std::setw(2) << std::setfill('0') << std::hex << long_frame_buf.getBufDataAt(buf_index, cnt) << std::endl;
                    } 
                    std::cout << "Receive CAN ID is: 0x" << std::setw(8) << std::setfill('0') << std::hex << long_frame_buf.getBufCanId(buf_index) << std::endl;
#endif
                    can_msg.ID = long_frame_buf.getBufCanId(buf_index);
                    can_msg.DataLen = long_frame_buf.getBufUsedLen(buf_index);
                    can_msg.Data.resize(can_msg.DataLen);
                    long_frame_buf.MemcpyDataFromBuf(buf_index, 0, &can_msg.Data[0], can_msg.DataLen);
                    long_frame_buf.FreeBuf(buf_index);
                }
            }
        }

    exit:
        return can_msg;
    }

    mrobot_driver_msgs::vci_can frame_construct( can::CanMsg * CMsg )
    {
        mrobot_driver_msgs::vci_can can_msg;

        CAN_ID_UNION id;
        CAN_DATA_UNION rx_buf;
        int buf_index;
        int seg_polo;
        int seg_num;

        for( int i = 0 ; i < CMsg->getLength(); i++ )
        {
            rx_buf.CanData[i] = CMsg->getAt(i);
        }

        seg_polo = rx_buf.CanData_Struct.SegPolo;
        seg_num = rx_buf.CanData_Struct.SegNum;

        if (seg_polo == ONLYONCE)
        {
            id.CANx_ID = CMsg->getID();
            std::cout << "short frame buf len:" << std::dec << CMsg->getLength() - 1 << std::endl;
            for(int cnt = 1; cnt < CMsg->getLength(); cnt++)
            {
                std::cout << "short frame buf data[" << cnt -1 << "] = 0x" << std::setw(2) << std::setfill('0') << std::hex <<  CMsg->getAt(cnt) << std::endl;		
            }

            can_msg.ID = CMsg->getID();
            can_msg.DataLen = CMsg->getLength() - 1;
            can_msg.Data.resize( CMsg->getLength() - 1);
            memcpy(&can_msg.Data[0], &rx_buf.CanData[1], CMsg->getLength() - 1);  
        }
        else //long frame
        {
            if(seg_polo == BEGIAN)
            {
                buf_index = long_frame_buf.GetTheBufById( CMsg->getID() );
                if( buf_index == CAN_BUF_NO_THIS_ID )
                {
                    buf_index = long_frame_buf.GetOneFreeBuf();
                }

                if((buf_index == CAN_LONG_BUF_FULL) || (buf_index >= CAN_LONG_BUF_NUM))
                {
                    std::cout << "LONG FRAME RCV BUF IS FULL! ! ! !" << std::endl;
                    goto exit;
                }
                long_frame_buf.MemcpyDataToBuf(buf_index, 0, rx_buf.CanData_Struct.Data, long_frame_buf.one_frame_data_length);
                long_frame_buf.addBufUsedLen( buf_index, long_frame_buf.one_frame_data_length );
                long_frame_buf.setBufCanId( buf_index, CMsg->getID() );
                //can_long_frame_buf->can_rcv_buf[buf_index].start_time = os_get_time();
            }
            else if((seg_polo == TRANSING) || (seg_polo == END))
            {
                buf_index = long_frame_buf.GetTheBufById( CMsg->getID() );
                if((buf_index == CAN_BUF_NO_THIS_ID) || (buf_index >= CAN_LONG_BUF_NUM))
                {
                    std::cout << "ERROR ! !" << std::endl;
                    goto exit;
                }
                //can_long_frame_buf->can_rcv_buf[buf_index].start_time = os_get_time();
                if(seg_polo == TRANSING)
                {
                    long_frame_buf.MemcpyDataToBuf(buf_index, seg_num*long_frame_buf.one_frame_data_length, rx_buf.CanData_Struct.Data, long_frame_buf.one_frame_data_length);
                    long_frame_buf.addBufUsedLen( buf_index, long_frame_buf.one_frame_data_length );
                }
                if(seg_polo == END)
                {
                    long_frame_buf.MemcpyDataToBuf(buf_index, seg_num*long_frame_buf.one_frame_data_length, rx_buf.CanData_Struct.Data, CMsg->getLength() - 1);
                    long_frame_buf.addBufUsedLen( buf_index, CMsg->getLength() - 1 ); 

                    for(int cnt = 0; cnt < long_frame_buf.getBufUsedLen(buf_index); cnt++)
                    {
                        std::cout << "long buf data[" << std::dec << cnt << "] = 0x" << std::setw(2) << std::setfill('0') << std::hex << long_frame_buf.getBufDataAt(buf_index, cnt) << std::endl;
                    } 
                    std::cout << "Receive CAN ID is: 0x" << std::setw(8) << std::setfill('0') << std::hex << long_frame_buf.getBufCanId(buf_index) << std::endl;
                    can_msg.ID = long_frame_buf.getBufCanId(buf_index);
                    can_msg.DataLen = long_frame_buf.getBufUsedLen(buf_index);
                    can_msg.Data.resize(can_msg.DataLen);
                    long_frame_buf.MemcpyDataFromBuf(buf_index, 0, &can_msg.Data[0], can_msg.DataLen);
                    long_frame_buf.FreeBuf(buf_index);
                }
            }
        }

    exit:
        return can_msg;
    }

    void frame_parser(const mrobot_driver_msgs::vci_can* can_msg, sendhandle_fn send_handle)
    {
        can::CanMsg CMsg;
        uint16_t t_len;
        uint16_t roundCount;
        uint8_t modCount;
        CAN_DATA_UNION TxMsg;

        t_len = can_msg->DataLen;
        roundCount = t_len/7;
        modCount = t_len%7;

        if(t_len <= 7)
        {
            TxMsg.CanData_Struct.SegPolo = ONLYONCE;

            CMsg.setLength( t_len + 1 );
            for( int i = 0; i < t_len; i++ )
            {
                CMsg.setAt(can_msg->Data[i], i + 1);
            }
            send_handle(&CMsg);
            return;
        }
        else 
        {
            for (int Num = 0; Num < roundCount; Num++)
            {
                //SET SEGPOLO				
                if( Num == 0)
                {
                    TxMsg.CanData_Struct.SegPolo = BEGIAN;
                }
                else
                {
                    TxMsg.CanData_Struct.SegPolo = TRANSING;
                }

                if( modCount == 0 && Num == roundCount-1)
                {
                    TxMsg.CanData_Struct.SegPolo = END;
                }

                TxMsg.CanData_Struct.SegNum = Num;
                memcpy(TxMsg.CanData_Struct.Data,&can_msg->Data[Num*7],7);
                for( int i = 0; i < 8; i++ )
                {
                    CMsg.setAt(TxMsg.CanData[i], i);
                }
                CMsg.setLength( 8 );

                send_handle(&CMsg);

                std::cout << "transmit_canmsg.DataLen:" << CMsg.getLength() << std::endl;
                for(int i = 0; i < CMsg.getLength(); i++)
                {
                    std::cout << "transmit_canmsg.Data[" << i << "]:0x" << std::setw(2) << std::hex << CMsg.getAt(i) << std::endl;
                }

                //TRANSMIT LAST MSG
                if( modCount !=0 && Num == roundCount-1 )
                {
                    Num++;
                    TxMsg.CanData_Struct.SegPolo = END;
                    TxMsg.CanData_Struct.SegNum = Num;
                    memcpy(TxMsg.CanData_Struct.Data,&can_msg->Data[Num*7],modCount);

                    for( int i = 0; i < modCount + 1; i++ )
                    {
                        CMsg.setAt(TxMsg.CanData[i], i);
                    }
                    CMsg.setLength( modCount + 1 );

                    send_handle(&CMsg);

                    std::cout << "transmit_canmsg.DataLen:d" << CMsg.getLength() << std::endl;
                    for(int i = 0; i < CMsg.getLength(); i++)
                    {
                        std::cout << "transmit_canmsg.Data[" << i << "]:0x" << std::setw(2) << std::hex << CMsg.getAt(i) << std::endl;
                    }
                }
            }
        }
        return;
    }

    //private:

};
#endif
