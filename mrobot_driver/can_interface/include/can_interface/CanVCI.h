#ifndef CANVCI_INCLUDEDEF_H
#define CANVCI_INCLUDEDEF_H
//-----------------------------------------------
//#include <boost/shared_ptr.hpp>
#include <can_interface/CanItf.h>
#include <can_interface/CanMsgBuffer.h>
extern "C" {
#include <can_interface/controlcan.h>
}
#include <thread>
//-----------------------------------------------
typedef struct can_channel {
    bool isChannelInitialized;
    int CanInd;
    int Baudrate; 
} can_channel_t;

typedef struct vci_can_dev {
    int DeviceType;
    int DeviceInd;
    bool isDeviceInitialized;
    can_channel_t can;
} device_t;

class CanVCI : public CanItf
{
public:
    // --------------- Interface
    CanVCI(int deviceType=VCI_USBCAN2, int deviceInd=0, int canInd=0, int baudrate=CANITFBAUD_500K, bool log_onoff=false);
    ~CanVCI();
    bool init_ret();
    void init();
    bool transmitMsg ( can::CanMsg CMsg, bool bBlocking = true );
    bool receiveMsg ( can::CanMsg* pCMsg );
    bool receiveMsgRetry ( can::CanMsg* pCMsg, int iNrOfRetry );
    bool receiveMsgTimeout ( can::CanMsg* pCMsg, int nMicroSecTimeout );

    size_t receiveMsgsToRxBuffer(void);
    bool readOneMsgFromRxBuffer(can::CanMsg* pCMsg);

    size_t transmitMsgsFromTxBuffer200Hz(void);
    bool writeOneMsgToTxBuffer( can::CanMsg* pCMsg);

    bool isObjectMode(){ return 0; }
    size_t getRxBufferSize( void );
    size_t getTxBufferSize( void );

private:
    // --------------- Types
//boost::shared_ptr<can::ThreadedSocketCANInterface> m_handle;
//    can::BufferedReader m_reader;

    class VCIDevice
    {
        public:
            int DeviceType;
            int DeviceInd;
            bool isInitialized;
            class CanChannel
            {
                public:
                    bool isInitialized;
                    int CanInd;
                    int Baudrate; 
            };
            CanChannel can_channel;
    };
    VCIDevice m_device;
    int mReserved;
    bool initCAN();
    can::CanMsgBuffer txMsgBuffer;
    can::CanMsgBuffer rxMsgBuffer;
//    void print_error ( const can::State& state );
    pthread_t _thread_tx;
    pthread_t _thread_rx;
    static void *thread_rx_handle(void* tmp);
    static void *thread_tx_handle(void* tmp);
    bool StartRun(void);
    bool is_log_on;
};
//-----------------------------------------------
#endif
