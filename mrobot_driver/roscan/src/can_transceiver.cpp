#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <fcntl.h>
#include <signal.h>
#include <mrobot_driver_msgs/vci_can.h>
#include <roscan/cm_can_id.h>
#include <roscan/can_long_frame.h>
#include <can_interface/CanVCI.h>

class can_transceiver
{
    public:
        CanVCI *can_itf;
        ros::NodeHandle n;
        ros::Publisher can_power_pub;
        ros::Publisher can_sensor_pub;
        ros::Publisher can_supersonic_pub;
        ros::Publisher can_micro_laser_pub;
        ros::Publisher can_base_pub;
        ros::Publisher can_sub_power_pub;
        ros::Publisher can_unknown_pub;
        ros::Publisher can_test_pub;
        ros::Publisher can_rfid_pub;
        ros::Publisher pub2starline_pub;
        ros::Publisher can_to_noah_powerboard_pub;
        ros::Publisher can_to_auto_charger_pub;
        ros::Publisher can_to_laser_pub;

        ros::Subscriber can_power_sub;
        ros::Subscriber can_sensor_sub;
        ros::Subscriber can_supersonic_sub;
        ros::Subscriber can_micro_laser_sub;
        ros::Subscriber can_base_sub;
        ros::Subscriber can_sub_power_sub;
        ros::Subscriber transmit_sub;
        ros::Subscriber can_test_sub;
        ros::Subscriber can_rfid_sub;
        ros::Subscriber noah_powerboard_to_can_sub;
        ros::Subscriber auto_charger_to_can_sub;
        ros::Subscriber laser_to_can_sub;

        ros::Publisher arm_move_pub;
        ros::Publisher head_move_pub;
        ros::Publisher camera_move_pub;
        ros::Publisher scan_move_pub;

        ros::Subscriber arm_sub;
        ros::Subscriber arm_motor_sub;
        ros::Subscriber head_sub;
        ros::Subscriber head_motor_sub;
        ros::Subscriber camera_sub;
        ros::Subscriber scanner_sub;

        ~can_transceiver()
        {
            delete can_itf;
        }

        can_transceiver(bool can_log_on= false, bool buffer_log_on = false)
        {
            is_log_on = can_log_on;
            can_itf = new CanVCI(4,0,0,2,buffer_log_on);
            can_power_pub = n.advertise<mrobot_driver_msgs::vci_can>("rx_power_node", 1000);//adam
            can_sensor_pub = n.advertise<mrobot_driver_msgs::vci_can>("rx_sensor_node", 1000);//kiqi
            can_supersonic_pub = n.advertise<mrobot_driver_msgs::vci_can>("can_to_ultrasonic", 1000);//kaka@2017/11/19
            can_micro_laser_pub = n.advertise<mrobot_driver_msgs::vci_can>("can_to_micro_laser", 1000);//kaka@2017/11/30
            can_base_pub = n.advertise<mrobot_driver_msgs::vci_can>("rx_base_node", 1000);//zero
            can_sub_power_pub = n.advertise<mrobot_driver_msgs::vci_can>("rx_sub_power_node", 1000);//kaka
            can_unknown_pub = n.advertise<mrobot_driver_msgs::vci_can>("rx_unknown_node", 1000);//adam
            can_test_pub = n.advertise<mrobot_driver_msgs::vci_can>("rx_test_node", 1000);//adam
            can_rfid_pub = n.advertise<mrobot_driver_msgs::vci_can>("rx_rfid_node", 1000);//adam@2017/09/21
            pub2starline_pub = n.advertise<mrobot_driver_msgs::vci_can>("rx_sensor_node", 1000);//milk@2017/10/24

            can_power_sub = n.subscribe("tx_power_node", 1000, &can_transceiver::canReceiveCallback, this);//adam
            can_sensor_sub = n.subscribe("tx_sensor_node", 1000, &can_transceiver::canReceiveCallback, this);//kiqi
            can_supersonic_sub = n.subscribe("ultrasonic_to_can", 1000, &can_transceiver::canReceiveCallback, this);//kaka@2017/11/19
            can_micro_laser_sub = n.subscribe("ultrasonic_to_can", 1000, &can_transceiver::canReceiveCallback, this);//kaka@2017/11/30
            can_base_sub = n.subscribe("tx_base_node", 1000, &can_transceiver::canReceiveCallback, this);//zero
            can_sub_power_sub = n.subscribe("tx_sub_power_node", 1000, &can_transceiver::canReceiveCallback, this);//kaka
            can_test_sub = n.subscribe("tx_test_node", 1000, &can_transceiver::canReceiveCallback, this);//
            can_rfid_sub = n.subscribe("tx_rfid_node", 1000, &can_transceiver::canReceiveCallback, this);//adam@2017/09/21
            noah_powerboard_to_can_sub = n.subscribe("noah_powerboard_to_can", 1000, &can_transceiver::canReceiveCallback, this);//Kaka@2017/10/30
            auto_charger_to_can_sub = n.subscribe("auto_charger_to_can", 1000, &can_transceiver::canReceiveCallback, this);//Kaka@2017/11/06
            laser_to_can_sub = n.subscribe("laser_to_can", 1000, &can_transceiver::canReceiveCallback, this);//Kaka@2017/11/16


            arm_move_pub = n.advertise<mrobot_driver_msgs::vci_can>("arm_control_node", 1000);
            head_move_pub = n.advertise<mrobot_driver_msgs::vci_can>("head_control_node", 1000);
            camera_move_pub = n.advertise<mrobot_driver_msgs::vci_can>("camera_control_node", 1000);
            scan_move_pub = n.advertise<mrobot_driver_msgs::vci_can>("scanner_control_node", 1000);
            can_to_noah_powerboard_pub = n.advertise<mrobot_driver_msgs::vci_can>("can_to_noah_powerboard", 1000);//Kaka@2017/10/30
            can_to_auto_charger_pub = n.advertise<mrobot_driver_msgs::vci_can>("can_to_auto_charger", 1000);//Kaka@2017/11/06
            can_to_laser_pub = n.advertise<mrobot_driver_msgs::vci_can>("can_to_laser", 1000);//Kaka@2017/11/16

            arm_sub = n.subscribe("arm_msgs", 1000, &can_transceiver::canReceiveCallback, this);
            arm_motor_sub = n.subscribe("arm_motor_control", 1000, &can_transceiver::canReceiveCallback, this);
            head_sub = n.subscribe("head_msgs", 1000, &can_transceiver::canReceiveCallback, this);
            head_motor_sub = n.subscribe("head_motor_control", 1000, &can_transceiver::canReceiveCallback, this);
            camera_sub = n.subscribe("camera_control", 1000, &can_transceiver::canReceiveCallback, this);
            scanner_sub = n.subscribe("scanner_control", 1000, &can_transceiver::canReceiveCallback, this);
        }

        mrobot_driver_msgs::vci_can receive_func( void )
        {
            can::CanMsg CMsg;
            mrobot_driver_msgs::vci_can can_msg;
            if( can_itf->readOneMsgFromRxBuffer(&CMsg) )
            {
                can_msg.ID = CMsg.getID();
                can_msg.DataLen = CMsg.getLength();
                can_msg.Data.resize(CMsg.getLength());
                for ( int i = 0; i < CMsg.getLength(); i++ )
                {
                    can_msg.Data[i] = CMsg.getAt(i);
                }
                if (is_log_on)
                {
                    ROS_INFO("received msg:");
                    CMsg.print_v();
                }
            }
            return can_msg;
        }

        void canReceiveCallback(const mrobot_driver_msgs::vci_can::ConstPtr& can_msg)
        {
            can::CanMsg CMsg;
            CMsg.setID(can_msg->ID);
            CMsg.setLength(can_msg->DataLen);
            for( int i = 0; i < can_msg->DataLen; i++)
            {
                CMsg.setAt(can_msg->Data[i], i);
            }
            can_itf->writeOneMsgToTxBuffer(&CMsg);
            if (is_log_on)
            {
                ROS_INFO("send msg:");
                CMsg.print_v();
            }
        }
        void update ()
        {
            mrobot_driver_msgs::vci_can can_msg;
            can_id_union can_id_u;
            ros::spinOnce();
            can_msg = receive_func();
            ros::spinOnce();
            can_id_u.can_id = can_msg.ID;
            if( can_id_u.can_id_stru.SrcMacID )
            {
                switch( can_id_u.can_id_stru.SrcMacID )
                {
#if 0
                    case 0x40://zero
                        can_base_pub.publish(can_msg);
                        break;
                    case 0x50:
                        can_test_pub.publish(can_msg);
                        can_power_pub.publish(can_msg);
                        break;
#endif
                    case 0x52://Kaka: noah powerboard
                        can_to_noah_powerboard_pub.publish(can_msg);
                        break;
                    case 0x6f:
                        can_rfid_pub.publish(can_msg);
                        can_test_pub.publish(can_msg);
                        break;

                   //case 0x60:
                        //if(can_id_u.can_id_stru.DstMacID == 1)
                            //pub2starline_pub.publish(can_msg);
                        //break;
                    case 0x60:
                    case 0x61:
                    case 0x62:
                    case 0x63:
                    case 0x64:
                    case 0x65:
                    case 0x66:
                    case 0x67:
                    case 0x68:
                    case 0x69:
                    case 0x6a:
                    case 0x6b:
                    case 0x6c:
                    case 0x6d:
                    case 0x6e:
                        can_supersonic_pub.publish(can_msg);
                        can_test_pub.publish(can_msg);//firmware update
                        break;
                    case 0x70:
                    case 0x71:
                    case 0x72:
                    case 0x73:
                    case 0x74:
                    case 0x75:
                    case 0x76:
                    case 0x77:
                    case 0x78:
                    case 0x79:
                    case 0x7a:
                    case 0x7b:
                    case 0x7c:
                    case 0x7d:
                        
                        can_micro_laser_pub.publish(can_msg);//firmware update
                        can_test_pub.publish(can_msg);//firmware update
                        break;
                    case 0xD1:
                        can_to_auto_charger_pub.publish(can_msg);
                        break;
#if 0
                    case 0x51:
                        can_sub_power_pub.publish(can_msg);
                        break;
                    case 0x20:
                        head_move_pub.publish(can_msg);
                        break;
                    case 0x21:
                        head_move_pub.publish(can_msg);
                        break;
                    case 0x22:
                        scan_move_pub.publish(can_msg);
                        break;
                    case 0x23:
                    case 0x24:
                        camera_move_pub.publish(can_msg);
                        break;
                    case 0x29:
                        arm_move_pub.publish(can_msg);
                        break;
                    case 0x2a:
                        arm_move_pub.publish(can_msg);
                        break;
                    case 0x2b:
                        arm_move_pub.publish(can_msg);
                        break;
                    case 0x2c:
                        arm_move_pub.publish(can_msg);
                        break;
                    case 0x30:
                        arm_move_pub.publish(can_msg);
                        break;
                    case 0x31:
                        arm_move_pub.publish(can_msg);
                        break;
                    case 0x32:
                        arm_move_pub.publish(can_msg);
                        break;
                    case 0x33:
                        arm_move_pub.publish(can_msg);
                        break;
#endif
                    default:
                        //can_unknown_pub.publish(can_msg);
                        break;
                }
            }
        }
    private:
        bool is_log_on;
};


void sigintHandler(int sig)
{
    ROS_INFO("killing on exit");
    ros::shutdown();
}

int main(int argc, char **argv)
{
    bool is_can_log_on;
    bool is_buffer_log_on;
    ros::init(argc, argv, "can_transceiver", ros::init_options::NoSigintHandler);

    ROS_INFO("start to create node...");
    ros::param::get("~can_log", is_can_log_on);
    ros::param::get("~buffer_log", is_buffer_log_on);
    can_transceiver *node = new can_transceiver(is_can_log_on, is_buffer_log_on);

    if( node->can_itf->init_ret() )
    {
        ROS_INFO("can devices init success...");
    }
    else
    {
        ROS_INFO("can devices init failed...");
        return -1;
    }

    signal(SIGINT, sigintHandler);
    ros::Rate loop_rate(1000);
    sleep(1);
    ROS_INFO("roscan node start work...");

    while (node->n.ok())
    {
        node->update();
        ros::spinOnce();
        if ( node->can_itf->getRxBufferSize() < 10 && node->can_itf->getTxBufferSize() < 10 )
        {
            loop_rate.sleep();
        }
    }

//    delete node;
    return 0;
}
