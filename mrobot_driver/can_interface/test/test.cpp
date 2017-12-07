
#include <can_interface/CanVCI.h>

int main(int argc, char **argv)
{
    CanVCI can_vci;
    can::CanMsg CMsg;
    if( !can_vci.init_ret() )
    {
        std::cout << "init can error" << endl;
        return -1;
    }
    int count;
    while(1)
    {
        count ++;
        if( count > 200)
        {
            count = 0;
        //    can_vci.receiveMsgsToRxBuffer();
        }
        if( can_vci.readOneMsgFromRxBuffer(&CMsg) )
        {
            CMsg.print();
        }
        usleep(1000);
    }
    return 0;
}
