#include <stdlib.h>
#include <iostream>
#include <can_interface/CanVCI.h>
#include <chrono>
#include <ctime>
#include <unistd.h>
#include <thread>

extern "C" {
#include <can_interface/controlcan.h>
}

CanVCI::CanVCI(int deviceType, int deviceInd, int canInd, int baudrate, bool log_onoff)
{
    m_device.isInitialized = false;
    m_device.can_channel.isInitialized = false;

    m_device.DeviceType = deviceType;
    m_device.DeviceInd = deviceInd;
    m_device.can_channel.CanInd = canInd;
    m_device.can_channel.Baudrate = baudrate;
    mReserved = 0;
    this->rxMsgBuffer.setMaxLen(10000);
    this->txMsgBuffer.setMaxLen(10000);
    is_log_on = log_onoff;
}

CanVCI::~CanVCI()
{
    pthread_join(_thread_rx, NULL);
    pthread_join(_thread_tx, NULL);
    if (m_device.isInitialized)
    {
        VCI_CloseDevice(m_device.DeviceType, m_device.DeviceInd);
        std::cout << "closed device:" << m_device.DeviceType << " index:" << m_device.DeviceInd << std::endl;
    }
    m_device.can_channel.isInitialized = false;
    m_device.isInitialized = false;
}

bool CanVCI::init_ret()
{
    bool ret = true;

    if(m_device.isInitialized)
    {
        std::cout << "can device:" << m_device.DeviceType << " index:" << m_device.DeviceInd << " already initialzed" << std::endl;
        return ret;
    }
    ret = VCI_OpenDevice(m_device.DeviceType, m_device.DeviceInd, mReserved);

    if ( ret != STATUS_OK )
    {
        std::cout << "open can device:" << m_device.DeviceType << " index:" << m_device.DeviceInd << " failed" << std::endl;
    }
    else
    {
        std::cout << "open can device:" << m_device.DeviceType << " index:" << m_device.DeviceInd << " success" << std::endl;
        ret = initCAN();
    }
    StartRun();

    return ret;
}

void CanVCI::init()
{
    bool ret = true;

    if(m_device.isInitialized)
    {
        std::cout << "can device:" << m_device.DeviceType << " index:" << m_device.DeviceInd << " already initialzed" << std::endl;
        return;
    }
    ret = VCI_OpenDevice(m_device.DeviceType, m_device.DeviceInd, mReserved);

    if ( ret != STATUS_OK )
    {
        std::cout << "open can device:" << m_device.DeviceType << " index:" << m_device.DeviceInd << " failed" << std::endl;
        exit(0);
    }
    else
    {
        std::cout << "open can device:" << m_device.DeviceType << " index:" << m_device.DeviceInd << " success" << std::endl;
        ret = initCAN();
    }
    StartRun();
}

bool CanVCI::initCAN()
{
    int ret;
    bool bRet = true;

    if(m_device.can_channel.isInitialized)
    {
        std::cout << "can already inited" << std::endl;
        return bRet;
    }

    VCI_INIT_CONFIG config;

    config.AccCode = 0;//refer api
    config.AccMask = 0xffffffff; // 0xffffffff: receive all
    config.Filter = 3;// 0/1:all frame type 2:only standard frames 3:only extend frames
    config.Mode = 0;// 0: normal  1: receive only 2:self loop test

    switch(m_device.can_channel.Baudrate)
    {
        case CANITFBAUD_1M:
            config.Timing0 = 0x00;
            config.Timing1 = 0x14;
            break;
        case CANITFBAUD_500K:
            config.Timing0 = 0x00;
            config.Timing1 = 0x1c;
            break;
        case CANITFBAUD_250K:
            config.Timing0 = 0x01;
            config.Timing1 = 0x1c;
            break;
        case CANITFBAUD_125K:
            config.Timing0 = 0x03;
            config.Timing1 = 0x1c;
            break;
        case CANITFBAUD_50K:
            config.Timing0 = 0x09;
            config.Timing1 = 0x1c;
            break;
        case CANITFBAUD_20K:
            config.Timing0 = 0x18;
            config.Timing1 = 0x1c;
            break;
        case CANITFBAUD_10K:
            config.Timing0 = 0x31;
            config.Timing1 = 0x1c;
            break;
        default:
            config.Timing0 = 0x00;
            config.Timing1 = 0x1c;
            break;
    }

    std::cout << "start to init can " << std::endl;
    ret = VCI_InitCAN(m_device.DeviceType, m_device.DeviceInd, m_device.can_channel.CanInd, &config);
    if( ret != STATUS_OK )
    {
        std::cout << "init can device:" << m_device.DeviceType << " index:" << m_device.DeviceInd << " channel:" << m_device.can_channel.CanInd << " failed!" << std::endl;
        return bRet = false;
    } 
    std::cout << "init can device:" << m_device.DeviceType << " index:" << m_device.DeviceInd << " channel:" << m_device.can_channel.CanInd << " success!" << std::endl;

    ret = VCI_StartCAN(m_device.DeviceType, m_device.DeviceInd, m_device.can_channel.CanInd);
    if( ret != STATUS_OK )
    {
        std::cout << "start can device:" << m_device.DeviceType << " index:" << m_device.DeviceInd << " channel:" << m_device.can_channel.CanInd << " failed!" << std::endl;
        return bRet = false;
    }
    std::cout << "start can device:" << m_device.DeviceType << " index:" << m_device.DeviceInd << " channel:" << m_device.can_channel.CanInd << " success!" << std::endl;

    m_device.can_channel.isInitialized = true;

    return bRet;
}

bool CanVCI::transmitMsg(can::CanMsg CMsg, bool bBlocking)
{
    bool bRet = true;

    if( !m_device.can_channel.isInitialized )
    {
        std::cout << "CanVCI::transmitMsg Error, can has not initialzed" << std::endl;
        return bRet = false;
    }
    VCI_CAN_OBJ vci_transmit_msg;
    vci_transmit_msg.ID = CMsg.getID();
    vci_transmit_msg.SendType = 0;//0:normal 1:single 2:normal self send 3:single self send
    vci_transmit_msg.RemoteFlag = 0;//0: data frame 1: remote frame
    vci_transmit_msg.ExternFlag = 1;//0:11 bit ID 1:29 bits ID
    vci_transmit_msg.DataLen = CMsg.getLength();
    for(int i = 0; i < vci_transmit_msg.DataLen; i++)
    {
        vci_transmit_msg.Data[i] = CMsg.getAt(i);
    }
    //VCI_Transmit() called period must more than 5ms
    if( 1 == VCI_Transmit(m_device.DeviceType, m_device.DeviceInd, 
                m_device.can_channel.CanInd, &vci_transmit_msg, 1) )
    {
#if __DEBUG__
        std::cout << "TCAN:" << m_device.DeviceInd << std::endl;
        std::cout << "TCAN ID: 0x" << hex << vci_transmit_msg.ID << std::endl;
        std::cout << "TCAN DataLen: " << vci_transmit_msg.DataLen << std::endl;
        for( int i = 0; i < vci_transmit_msg.DataLen; i++ )
        {
            std::cout << "TCAN Data [" << i << "]: 0x" << hex << vci_transmit_msg.Data[i] << std::endl;
        }
#endif
        bRet = true;
    }
    else
    {
        std::cout << "CanVCI::transmitMsg An error occured while sending..." << std::endl;
        bRet = false;
    }
    return bRet;
}

bool CanVCI::writeOneMsgToTxBuffer(can::CanMsg* pCMsg)
{
    return this->txMsgBuffer.handleCanMsg(*pCMsg);
}

size_t CanVCI::getTxBufferSize( void )
{
    return this->txMsgBuffer.getBufferSize();
}

size_t CanVCI::transmitMsgsFromTxBuffer200Hz(void)
{
    int nTransmitMsgs;

    if( !m_device.can_channel.isInitialized )
    {
        std::cout << "CanVCI::transmitMsg Error, can has not initialzed" << std::endl;
        return 0;
    }
    nTransmitMsgs = this->txMsgBuffer.getBufferSize();

    if( nTransmitMsgs == 0 )
    {
        return 0;
    }
    else if( nTransmitMsgs > 48 )
    {
         nTransmitMsgs = 48;
    }
    
    if (is_log_on)
    {
        std::cout << "tx buffer size: " << std::dec << this->txMsgBuffer.getBufferSize() << std::endl;
    }
    VCI_CAN_OBJ vci_transmit_msg[nTransmitMsgs];
    can::CanMsg CMsg;
    for ( int i = 0; i < nTransmitMsgs; i++ )
    {
        this->txMsgBuffer.read(&CMsg);
        vci_transmit_msg[i].ID = CMsg.getID();
        vci_transmit_msg[i].SendType = 0;//0:normal 1:single 2:normal self send 3:single self send
        vci_transmit_msg[i].RemoteFlag = 0;//0: data frame 1: remote frame
        vci_transmit_msg[i].ExternFlag = 1;//0:11 bit ID 1:29 bits ID
        vci_transmit_msg[i].DataLen = CMsg.getLength();
        for(int j = 0; j < vci_transmit_msg[i].DataLen; j++)
        {
            vci_transmit_msg[i].Data[j] = CMsg.getAt(j);
        }
#if __DEBUG__
        std::cout << "TCAN:" << m_device.DeviceInd << std::endl;
        std::cout << "TCAN ID: 0x" << hex << vci_transmit_msg[i].ID << std::endl;
        std::cout << "TCAN DataLen: " << vci_transmit_msg[i].DataLen << std::endl;
        for( int j = 0; j < vci_transmit_msg[i].DataLen; j++ )
        {
            std::cout << "TCAN Data [" << j << "]: 0x" << hex << vci_transmit_msg[i].Data[j] << std::endl;
        }
#endif
    }
    //VCI_Transmit() called period must more than 5ms
    int readTransmit = VCI_Transmit(m_device.DeviceType, m_device.DeviceInd, 
                m_device.can_channel.CanInd, vci_transmit_msg, nTransmitMsgs);
    if ( nTransmitMsgs != readTransmit )
    {
        std::cout << "CanVCI::transmitMsg An error occured while sending, read sent: " << \
            std::dec << readTransmit << " msgs" << std::endl;
    }
    return readTransmit;
}

bool CanVCI::receiveMsg(can::CanMsg* pCMsg)
{
    bool bRet = false;

    if( !m_device.can_channel.isInitialized )
    {
        std::cout << "CanVCI::receiveMsg Error, can has not initialzed" << std::endl;
        return bRet;
    }

    VCI_CAN_OBJ rec;
    int rec_len = 0;

    //VCI_Receive() called period must more than 5ms
    rec_len = VCI_Receive(m_device.DeviceType, m_device.DeviceInd, m_device.can_channel.CanInd, &rec, 1, -1);
    if( rec_len > 0)
    {
        pCMsg->setID(rec.ID);
        pCMsg->setLength(rec.DataLen);
        pCMsg->set(rec.Data[0], rec.Data[1], rec.Data[2], rec.Data[3], rec.Data[4],
                rec.Data[5], rec.Data[6], rec.Data[7]);
        bRet = true;
    }
    return bRet;
}

size_t CanVCI::receiveMsgsToRxBuffer(void)
{
    int rec_len = 0;

    if( !m_device.can_channel.isInitialized )
    {
        std::cout << "CanVCI::receiveMsg Error, can has not initialzed" << std::endl;
        return rec_len;
    }

    VCI_CAN_OBJ rec[2500];

    //VCI_Receive() called period must more than 5ms
    rec_len = VCI_Receive(m_device.DeviceType, m_device.DeviceInd, m_device.can_channel.CanInd, rec, 2500, 0);
    if( rec_len > 0)
    {
        can::CanMsg pCMsg;
        for( int i = 0; i < rec_len; i++ )
        {
            pCMsg.setID(rec[i].ID);
            pCMsg.setLength(rec[i].DataLen);
            pCMsg.set(rec[i].Data[0], rec[i].Data[1], rec[i].Data[2], rec[i].Data[3], rec[i].Data[4],
                rec[i].Data[5], rec[i].Data[6], rec[i].Data[7]);
            this->rxMsgBuffer.handleCanMsg(pCMsg);
        }
        if (is_log_on)
        {
            std::cout << "rx buffer size: " << std::dec << this->rxMsgBuffer.getBufferSize() << std::endl;
        }
    }
    return rec_len;
}

bool CanVCI::readOneMsgFromRxBuffer(can::CanMsg* pCMsg)
{
    return this->rxMsgBuffer.read(pCMsg);
}

size_t CanVCI::getRxBufferSize( void )
{
    return this->rxMsgBuffer.getBufferSize();
}

bool CanVCI::receiveMsgRetry(can::CanMsg* pCMsg, int iNrOfRetry)
{
    int i, rec_len; 
    bool bRet = true;

    if( !m_device.can_channel.isInitialized )
    {
        std::cout << "CanVCI::receiveMsgRetry Error, can has not initialzed" << std::endl;
        return bRet = false;
    }
    VCI_CAN_OBJ rec;
    i = 0;
    do {
        //VCI_Receive() called period must more than 5ms
        rec_len = VCI_Receive(m_device.DeviceType, m_device.DeviceInd, m_device.can_channel.CanInd, &rec, 1, 0);
        if( !rec_len )
            break;
        i++;
        usleep(10000);
    } while( i < iNrOfRetry );

    if( !rec_len )
    {
        std::cout << "CanVCI::receiveMsgRetry, no msg received" << std::endl;
        pCMsg->set(0, 0, 0, 0, 0, 0, 0, 0);
        bRet = false;
    }
    else
    {
        pCMsg->setID(rec.ID);
        pCMsg->setLength(rec.DataLen);
        pCMsg->set(rec.Data[0], rec.Data[1], rec.Data[2], rec.Data[3], rec.Data[4],
                rec.Data[5], rec.Data[6], rec.Data[7]);
    }

    return bRet;
}

bool CanVCI::receiveMsgTimeout(can::CanMsg* pCMsg, int mMicroSecTimeout)
{
    int rec_len;
    bool bRet = true;

    if( !m_device.can_channel.isInitialized )
    {
        std::cout << "CanVCI::receiveMsgRetry Error, can has not initialzed" << std::endl;
        return bRet = false;
    }
    VCI_CAN_OBJ rec;
    //VCI_Receive() called period must more than 5ms
    rec_len = VCI_Receive(m_device.DeviceType, m_device.DeviceInd, m_device.can_channel.CanInd, &rec, 1, mMicroSecTimeout);
    if( !rec_len )
    {
        std::cout << "CanVCI::receiveMsgTimeout, no msg received" << std::endl;
        pCMsg->set(0, 0, 0, 0, 0, 0, 0, 0);
        bRet = false;
    }
    else
    {
        pCMsg->setID(rec.ID);
        pCMsg->setLength(rec.DataLen);
        pCMsg->set(rec.Data[0], rec.Data[1], rec.Data[2], rec.Data[3], rec.Data[4],
                rec.Data[5], rec.Data[6], rec.Data[7]);
    }

    return bRet;
}

void* CanVCI::thread_rx_handle(void* tmp)
{
    CanVCI *p = (CanVCI *)tmp;
//    using chrono::system_clock;
//    typedef chrono::duration<int,std::milli> milliseconds_type;

//    milliseconds_type dration(5);
//    system_clock::time_point last_tp = system_clock::now();

    std::cout << "start can rx thread" << std::endl;
    for (;;)
    {
//        while( system_clock::now() - last_tp < dration ) { usleep(1000);}
//        last_tp = system_clock::now();
//        std::time_t tt = system_clock::to_time_t(last_tp);
//        cout << "rx test dration " << ctime(&tt); 
        usleep(5000);
        p->receiveMsgsToRxBuffer();
    }
    return NULL;
}

void* CanVCI::thread_tx_handle(void* tmp)
{
    CanVCI *p = (CanVCI *)tmp;
//    using chrono::system_clock;
//    typedef chrono::duration<int,std::milli> milliseconds_type;

//    milliseconds_type dration(5);
//    system_clock::time_point last_tp = system_clock::now();

    std::cout << "start can tx thread" << std::endl;
    for (;;)
    {
//        while( system_clock::now() - last_tp < dration ) { usleep(1000);}
//        last_tp = system_clock::now();
//        std::time_t tt = system_clock::to_time_t(last_tp);
 //       cout << "tx test dration " << ctime(&tt); 
        usleep(5000);
        p->transmitMsgsFromTxBuffer200Hz();
    }
    return NULL;
}

bool CanVCI::StartRun(void)
{
    if (pthread_create(&_thread_rx, NULL, this->thread_rx_handle, this))
    {
        std::cout << "create can rx thread error" << std::endl;
        return false;
    }
    if (pthread_create(&_thread_tx, NULL, this->thread_tx_handle, this))
    {
        std::cout << "create can tx thread error" << std::endl;
        return false;
    }
    return true;
}
