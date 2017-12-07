#include <ros/ros.h>
#include <std_msgs/String.h>

#include <mrobot_driver_msgs/vci_can.h>
//#include "cm_can_id.h"
#include "can_powerboard.h"
#include "can_sub_pb.h"
//#include "can_protocol.h"
#include <roscan/can_long_frame.h>
#include <string>

#include "json.hpp"
using json = nlohmann::json;

using namespace std;
class SubPowerNode
{
    public:
        can_long_frame long_frame;

        SubPowerNode()
        {
           sub_powerboard_pub = n.advertise<mrobot_driver_msgs::vci_can>("tx_sub_power_node", 1000);
           sub_powerboard_sub = n.subscribe("rx_sub_power_node", 1000, &SubPowerNode::canReceivedCallback,this);

            to_app_pub = n.advertise<std_msgs::String>("/app_vice_power_sub", 1000);
            from_app_sub= n.subscribe("/app_vice_power_pub", 1000, &SubPowerNode::fromAppReceivedCallback, this);
           // this->total_receive = 0;
           total_receive = 0;
        }
        void fromAppReceivedCallback(const std_msgs::String::ConstPtr &msg);
        void canReceivedCallback(const mrobot_driver_msgs::vci_can::ConstPtr& msg);
        void ReadAdcData( void );
        void SetRkState(uint8_t head_rk,uint8_t chest_rk);
//        void readPowerSystemAndVbat( uint8_t group );
//        void ReadModuleStatus( uint8_t group );
//        void ReadFaultStatus( uint8_t group );
        void ReadVersionInfo(uint8_t type );
		void ReadRkState(void);
        void ReadModuleState(uint8_t group_num);
        void SetModeluState(uint8_t group_num, uint32_t state, uint8_t on_off);
        void TestFunc(void);
    private:
        ros::NodeHandle n;
        ros::Publisher sub_powerboard_pub;
        ros::Subscriber sub_powerboard_sub;
        ros::Publisher to_app_pub;
        ros::Subscriber from_app_sub;

        unsigned int total_receive;
        char fw_version[20];
        char hw_version[10];
        char pl_version[10];
        int adc_select;
        uint32_t m_state, m_state2;
        
        json j;
        void publish_json_msg_to_app( const nlohmann::json j_msg);
};


void SubPowerNode::publish_json_msg_to_app( const nlohmann::json j_msg)
{
    std_msgs::String pub_json_msg;
    std::stringstream ss;

    ss.clear();
    ss << j_msg;
    pub_json_msg.data = ss.str();
    this->to_app_pub.publish(pub_json_msg);
    std::cout << std::setw(4) << std::setfill(' ') << j_msg << std::endl;
}

void SubPowerNode::fromAppReceivedCallback(const std_msgs::String::ConstPtr &msg)
{
    auto j = json::parse(msg->data.c_str());
    if( j.is_object() )
    {
        if ( j.find("pub_name") != j.end() )
        {
            if ( j["pub_name"] == "get_module_state" )
            {
                ReadModuleState(1);
                ReadModuleState(2);
            }
            else if ( j["pub_name"] == "set_module_state" )
            {
                if ( j["data"]["dev_name"] == "led_a4_printer" )
                {
                    if ( j["data"]["set_state"] == true )
                    {
                        SetModeluState(2, led_a4_printer, 1);
                    }
                    else
                    {
                        SetModeluState(2, led_a4_printer, 0);
                    }
                }
                else if ( j["data"]["dev_name"] == "led_card_finger" )
                {
                    if ( j["data"]["set_state"] == true )
                    {
                        SetModeluState(2, led_card_finger, 1);
                    }
                    else
                    {
                        SetModeluState(2, led_card_finger, 0);
                    }
                }
                else if ( j["data"]["dev_name"] == "led_ticket_printer" )
                {
                    if ( j["data"]["set_state"] == true )
                    {
                        SetModeluState(2, led_ticket_printer, 1);
                    }
                    else
                    {
                        SetModeluState(2, led_ticket_printer, 0);
                    }
                }
                else if ( j["data"]["dev_name"] == "led_card_machine" )
                {
                    if ( j["data"]["set_state"] == true )
                    {
                        SetModeluState(2, led_card_machine, 1);
                    }
                    else
                    {
                        SetModeluState(2, led_card_machine, 0);
                    }
                }
                else
                {
                    std::cout << "not support dev_name currently: " << j["data"]["dev_name"] << std::endl;
                }
                this->j.clear();
                this->j = {
                    {"sub_name", "set_module_state"},
                    {"data", j["data"]},
                    {"error_code", 0},
                };
                this->publish_json_msg_to_app(this->j);
            }
            else if ( j["pub_name"] == "sw_version" )
            {
                ReadVersionInfo(1);
            }
            else if ( j["pub_name"] == "hw_version" )
            {
                ReadVersionInfo(2);
            }
            else if ( j["pub_name"] == "get_currents" )
            {
                adc_select = 1;
                ReadAdcData();
            }
            else if ( j["pub_name"] == "get_temperature" )
            {
                adc_select = 2;
                ReadAdcData();
            }
            else if ( j["pub_name"] == "get_voltage" )
            {
                adc_select = 3;
                ReadAdcData();
            }
            else
            {
                std::cout << "not found pub_name: " << j["pub_name"] << std::endl;
            }
        }
        else
        {
            std::cout << "not found contain \"pub_name\" in json object" << std::endl;
        }
    }
    else
    {
        std::cout << "msg is not a json object" << std::endl;
    }
    std::cout << std::setw(4) << std::setfill(' ') << j << std::endl;
}

void SubPowerNode::canReceivedCallback(const mrobot_driver_msgs::vci_can::ConstPtr& c_msg)
{
    mrobot_driver_msgs::vci_can can_msg;
    mrobot_driver_msgs::vci_can long_msg;
	CAN_ID_UNION id;

    long_msg = long_frame.frame_construct(c_msg);
    mrobot_driver_msgs::vci_can* msg = &long_msg;
    if( msg->ID == 0 )
    {
        return;
    }

    this->total_receive ++;
    can_msg.ID = msg->ID;
    id.CANx_ID = can_msg.ID;
    can_msg.DataLen = msg->DataLen;
    if(id.CanID_Struct.SrcMACID != CAN_SUB_PB_DST_ID)
    {
        return ;
    }
#if 0
    ROS_INFO("sub_power_board received No.%d", this->total_receive);
    ROS_INFO("can_msg.ID = 0x%08x", can_msg.ID);
    ROS_INFO("can_msg.DataLen = %d", can_msg.DataLen);
#endif
    can_msg.Data.resize(can_msg.DataLen);
#if 0
    ROS_INFO("source ID is 0x%x",id.CanID_Struct.SourceID);
    ROS_INFO("Src MAC ID is 0x%x",id.CanID_Struct.SrcMACID);
    ROS_INFO("Dst MAC ID is 0x%x",id.CanID_Struct.DestMACID);
#endif
    
    if(id.CanID_Struct.SrcMACID != CAN_SUB_PB_DST_ID) 
    {
//        ROS_INFO("NOT SUB_PB CAN ID!");
        return ;
    }

    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_MODULE_STATE)
    {
       if( msg->DataLen > 0 && msg->Data[0] == 2 )//read
       {
            if( msg->Data[1] == 1 )
            {
                m_state = *(uint32_t*)(&msg->Data[2]);
                ROS_INFO("module state :0x%x",m_state);   
                return;
            }
            if( msg->Data[1] == 2 )
            {
                m_state2 = *(uint32_t*)(&msg->Data[2]);
                ROS_INFO("module state2 :0x%x",m_state2);   
            }
            this->j.clear();
            this->j = {
                {"sub_name", "get_module_state"},
                {"data", {
                            {"5v_right_hand_motor", (bool)(m_state & right_hand_motor_5v)},
                            {"24v_head_motor", (bool)(m_state & head_motor_24v)},
                            {"5v_keyboard", (bool)(m_state & bar_code_reader_5v)},
                            {"5v_code_board", (bool)(m_state & code_board_5v)},
                            {"5v_left_hand_motor", (bool)(m_state & left_hand_motor_5v)},
                            {"5v_card_reader", (bool)(m_state & card_reader_5v)},
                            {"5v_head", (bool)(m_state & head_motor_5v)},
                            {"24v_right_hand_motor", (bool)(m_state & right_hand_motor_24v)},
                            {"12v_head_rk", (bool)(m_state & rk_head)},
                            {"12v_chest_rk", (bool)(m_state & rk_chest)},
                            {"24v_left_hand_motor", (bool)(m_state & left_hand_motor_24v)},
                            {"12v_audio_pa", (bool)(m_state & audio_pa_12v)},
                            {"5v_repair_board", (bool)(m_state & repair_board_5v)},
                            {"5v_touch_board", (bool)(m_state & touch_board_5v)},
                            {"12v_switch", (bool)(m_state & switch_12v)},
                            {"12v_router", (bool)(m_state & router_12v)},
                            {"12v_card_reader", (bool)(m_state & card_reader_12v)},
                            {"5v_reserve", (bool)(m_state & reserve_5v)},
                            {"5v_led", (bool)(m_state & led_boards_5v)},
                            {"12v_dcdc", (bool)(m_state & dcdc_12v)},
                            {"5v_dcdc", (bool)(m_state & dcdc_5v)},
                            {"led_card_finger", (bool)(m_state2 & led_card_finger)},
                            {"led_a4_printer", (bool)(m_state2 & led_a4_printer)},
                            {"led_ticket_printer", (bool)(m_state2 & led_ticket_printer)},
                            {"led_card_machine", (bool)(m_state2 & led_card_machine)},
                         }
                }
            };
            this->publish_json_msg_to_app(this->j);
       }
    }
    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_READ_VERSION)
    {
		if((msg->Data[0] == 1) || (msg->Data[0] == 2) || (msg->Data[0] == 3))
		{
			if(msg->Data[0] == 1)//software version info
			{
                ROS_INFO("software version :");  
                memcpy(this->fw_version, (const void*)&msg->Data[1],msg->DataLen);
                this->j.clear();
                this->j = {
                    {"sub_name", "sw_version"},
                    {"data", this->fw_version},
                };
                this->publish_json_msg_to_app(this->j);
			}
			else if(msg->Data[0] == 2)//protocol version info
			{
				ROS_INFO("protocol version :");  
                memcpy(this->pl_version, (const void*)&msg->Data[1],msg->DataLen);
                this->j.clear();
                this->j = {
                    {"sub_name", "pl_version"},
                    {"data", this->pl_version},
                };
                this->publish_json_msg_to_app(this->j);
			}
			else if(msg->Data[0] == 3)//hardware version info
			{
				ROS_INFO("hardware version :");  
                memcpy(this->hw_version, (const void*)&msg->Data[1],msg->DataLen);
                this->j.clear();
                this->j = {
                    {"sub_name", "hw_version"},
                    {"data", this->hw_version},
                };
                this->publish_json_msg_to_app(this->j);
			}
		
#if 0			
			for(uint8_t i = 1; i < msg->DataLen; i++)
			{
				ROS_INFO("%c",msg->Data[i]);
			}
#endif
		}
	
       
    }
    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_READ_RK_STATE)
    {
        if(msg->DataLen == 3)
        {
           if(msg->Data[0] == 1)//read 
           {
               ROS_INFO("HEAD RK state is %d",msg->Data[1]);
               ROS_INFO("CHEST RK state is %d",msg->Data[2]);
           }
           if(msg->Data[0] == 2)//write 
           {
               ROS_INFO("HEAD RK state is %d",msg->Data[1]);
               ROS_INFO("CHEST RK state is %d",msg->Data[2]);
           }
                                                                 
           this->j.clear();
           this->j = {
               {"sub_name", "get_rk_state"},
               {"data", {
                            {"head_rk_state",(bool)msg->Data[1] },
                            {"chest_rk_state",(bool)msg->Data[2] },
                        }
               },
           };
           this->publish_json_msg_to_app(this->j);
        }
    }
	if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_READ_ADC_DATA)
	{
		if(msg->DataLen == sizeof(voltageData_t))
		{
			voltageData_t voltageConvert_ram;
			voltageData_t *voltageConvert = &voltageConvert_ram;
			memcpy(voltageConvert, &msg->Data[0], sizeof(voltageData_t));
#if 0

		    ROS_INFO("5v right hand motor currents        %d",voltageConvert->c_5V_right_hand_motor);
		    ROS_INFO("24v hd camera currents              %d",voltageConvert->c_24V_hd_camera);
		    ROS_INFO("24v head motor currents             %d",voltageConvert->c_24V_head_motor);
		    ROS_INFO("24v camera motor currents           %d",voltageConvert->c_24V_camera);
		    ROS_INFO("5v keyboard currents                %d",voltageConvert->c_5V_keyboard);
		    ROS_INFO("5v code board currents              %d",voltageConvert->c_5V_code);
		    ROS_INFO("5v left hand motor currents         %d",voltageConvert->c_5V_left_hand_motor);
		    ROS_INFO("5v card reader currents             %d",voltageConvert->c_5V_card_reader);
		    ROS_INFO("5v head currents                    %d",voltageConvert->c_5V_head);
		    ROS_INFO("24v right hand motor currents       %d",voltageConvert->c_24V_right_hand_motor);
		    ROS_INFO("12v head rk currents                %d",voltageConvert->c_12V_head_rk);
		    ROS_INFO("12v chest rk currents               %d",voltageConvert->c_12V_chest_rk);
		    ROS_INFO("24v left hand motor currents        %d",voltageConvert->c_24V_left_hand_motor);
		    ROS_INFO("12v audio pa currents               %d",voltageConvert->c_12V_audio_pa);
		    ROS_INFO("5v repair board currents            %d",voltageConvert->c_5V_repair);
		    ROS_INFO("5v touch board currents             %d",voltageConvert->c_5V_touch);
		    ROS_INFO("12v switch currents                 %d",voltageConvert->c_12V_switch);
		    ROS_INFO("12v router currents                 %d",voltageConvert->c_12V_router);
		    ROS_INFO("vbus currents                       %d",voltageConvert->c_vbus);
		    ROS_INFO("12 card reader currents             %d",voltageConvert->c_12V_card_reader);
		    ROS_INFO("12v temperature                     %d",voltageConvert->temp_12V_ts);
		    ROS_INFO("5v temperature                      %d",voltageConvert->temp_5V_ts);
		    ROS_INFO("air temperature                     %d",voltageConvert->temp_air_ts);
		    ROS_INFO("5v head touch currents              %d",voltageConvert->c_5V_head_touch);
		    ROS_INFO("12v all currents                    %d",voltageConvert->c_12V_all);
		    ROS_INFO("5v all currents                     %d",voltageConvert->c_5V_all);
		    ROS_INFO("5v head led currents                %d",voltageConvert->c_5V_head_led);
		    ROS_INFO("12v voltage                         %d",voltageConvert->v_12V);
		    ROS_INFO("5v voltage                          %d",voltageConvert->v_5V);
		    ROS_INFO("vbat voltage                        %d",voltageConvert->v_bat);
		    ROS_INFO("5v reserve currents                 %d",voltageConvert->c_5V_reserve);
		    ROS_INFO("5v led currents                     %d",voltageConvert->c_5V_led);
		    ROS_INFO("5v camera currents                  %d",voltageConvert->c_5V_camera);
		    ROS_INFO("5v hd camera currents               %d",voltageConvert->c_5V_hd_camera);
#endif
            switch (adc_select)
            {
                case 1:
                    this->j.clear();
                    this->j = {
                        {"sub_name", "get_current_ma"},
                        {"data", {
                                     {"5v_right_hand_motor", voltageConvert->c_5V_right_hand_motor},
                                     {"24v_hd_camera", voltageConvert->c_24V_hd_camera},
                                     {"24v_head_motor", voltageConvert->c_24V_head_motor},
                                     {"24v_camera_motor", voltageConvert->c_24V_camera},
                                     {"5v_keyboard", voltageConvert->c_5V_keyboard},
                                     {"5v_code_board", voltageConvert->c_5V_code},
                                     {"5v_left_hand_motor", voltageConvert->c_5V_left_hand_motor},
                                     {"5v_card_reader", voltageConvert->c_5V_card_reader},
                                     {"5v_head", voltageConvert->c_5V_head},
                                     {"24v_right_hand_motor", voltageConvert->c_24V_right_hand_motor},
                                     {"12v_head_rk", voltageConvert->c_12V_head_rk},
                                     {"12v_chest_rk", voltageConvert->c_12V_chest_rk},
                                     {"24v_left_hand_motor", voltageConvert->c_24V_left_hand_motor},
                                     {"12v_audio_pa", voltageConvert->c_12V_audio_pa},
                                     {"5v_repair_board", voltageConvert->c_5V_repair},
                                     {"5v_touch_board", voltageConvert->c_5V_touch},
                                     {"12v_switch", voltageConvert->c_12V_switch},
                                     {"12v_router", voltageConvert->c_12V_router},
                                     {"vbus", voltageConvert->c_vbus},
                                     {"12v_card_reader", voltageConvert->c_12V_card_reader},
                                     {"5v_head_touch", voltageConvert->c_5V_head_touch},
                                     {"12v_all", voltageConvert->c_12V_all},
                                     {"5v_all", voltageConvert->c_5V_all},
                                     {"5v_head_led", voltageConvert->c_5V_head_led},
                                     {"5v_reserve", voltageConvert->c_5V_reserve},
                                     {"5v_led", voltageConvert->c_5V_led},
                                     {"5v_camera", voltageConvert->c_5V_camera},
                                     {"5v_hd_camera", voltageConvert->c_5V_hd_camera},
                                 }
                        },
                    };
                    this->publish_json_msg_to_app(this->j);
                    break;

                case 2:
                    this->j.clear();
                    this->j = {
                        {"sub_name", "get_temperature"},
                        {"data", {
                                     {"12v_dcdc_temperature", voltageConvert->temp_12V_ts},
                                     {"5v_dcdc_temperature", voltageConvert->temp_5V_ts},
                                     {"air_temperature", voltageConvert->temp_air_ts},
                                 }
                        },
                    };
                    this->publish_json_msg_to_app(this->j);
                    break;

                case 3:
                    this->j.clear();
                    this->j = {
                        {"sub_name", "get_voltage_mv"},
                        {"data", {
                                     {"12v_dcdc", voltageConvert->v_12V},
                                     {"5v_dcdc", voltageConvert->v_5V},
                                     {"battery", voltageConvert->v_bat},
                                 }
                        },
                    };
                    this->publish_json_msg_to_app(this->j);
                    break;
            }
		}
	}

    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sub_powerboard_node");

    SubPowerNode sub_power_node;

    uint32_t count;

    ros::Rate loop_rate(1000);
    
    uint8_t flag = 5;
    int cmd_num = 5;
    sleep(2);
    while (ros::ok())
    {
        count ++;
        if( count >= 1000 )
        {
            count = 0;
            switch( cmd_num )
            {
                case 1:
//                    sub_power_node.SetModeluState(1, led_boards_5v, 0);
                    sub_power_node.SetModeluState(2, led_card_finger, 0);
                    sub_power_node.SetModeluState(2, led_a4_printer, 0);
                    sub_power_node.SetModeluState(2, led_ticket_printer, 0);
                    sub_power_node.SetModeluState(2, led_card_machine, 0);
                    sub_power_node.ReadModuleState(2);
                    //sub_power_node.ReadAdcData();
                    cmd_num = 2;
                    break;
                case 2:
//                    sub_power_node.SetModeluState(1, led_boards_5v, 1);
                    sub_power_node.SetModeluState(2, led_card_finger, 1);
                    sub_power_node.SetModeluState(2, led_a4_printer, 1);
                    sub_power_node.SetModeluState(2, led_ticket_printer, 1);
                    sub_power_node.SetModeluState(2, led_card_machine, 1);
                    sub_power_node.ReadModuleState(2);
                    cmd_num = 1;
                    break;
                case 3:
                    //sub_power_node.SetModeluState(1, dcdc_12v, 1);
                    //sub_power_node.ReadModuleState();
        			//sub_power_node.ReadRkState();
                    cmd_num = 1;
                    break;
            }
//			sub_power_node.SetRkState(1, 1);
//            sub_power_node.TestFunc();
        }
        if ( flag )
        {
            flag = 0;
            sub_power_node.ReadVersionInfo(1);
            sub_power_node.SetModeluState(1, dcdc_5v | dcdc_12v , 1);
//            sub_power_node.ReadVersionInfo(2);
//            sub_power_node.ReadVersionInfo(3);
//            sub_power_node.SetRkState(1, 1);
            
//            sub_power_node.ReadAdcData();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("powerboard node exit");

    return 0;
}

void SubPowerNode::ReadAdcData(void)
{
    mrobot_driver_msgs::vci_can can_msg;
    CAN_ID_UNION id;

    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_READ_ADC_DATA;
    id.CanID_Struct.SrcMACID = CAN_SUB_PB_SRC_ID;
    id.CanID_Struct.DestMACID = CAN_SUB_PB_DST_ID;
	id.CanID_Struct.FUNC_ID = 0x02;
	id.CanID_Struct.ACK = 0; 
	id.CanID_Struct.res = 0;
//    ROS_INFO("build can.ID = 0x%08x", id.CANx_ID);
    can_msg.ID = id.CANx_ID;
    can_msg.DataLen = 2;
    can_msg.Data.resize(2);
    can_msg.Data[0]  = 0x00;
    can_msg.Data[1]  = 0x00;
    this->sub_powerboard_pub.publish(can_msg);
}

void SubPowerNode::ReadRkState(void)
{
//   cm_can_id_t canid;
   mrobot_driver_msgs::vci_can can_msg;
//   CM_CAN_ID  can_id;
   CAN_ID_UNION id;

   memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_READ_RK_STATE;
    id.CanID_Struct.SrcMACID = CAN_SUB_PB_SRC_ID;
    id.CanID_Struct.DestMACID = CAN_SUB_PB_DST_ID;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 0;
    id.CanID_Struct.res = 0;
//    ROS_INFO("Get RK State can.ID = 0x%08x", id.CANx_ID);
    can_msg.ID = id.CANx_ID;
    can_msg.DataLen = 4;
	can_msg.Data.resize(4);
	can_msg.Data[0] = 0x00;
	can_msg.Data[1] = 1;
    can_msg.Data[2] = 0;
    can_msg.Data[3] = 0;
    //can_msg.ID = 0x00005678;
//  can_msg.ID = id.CANx_ID;
   this->sub_powerboard_pub.publish(can_msg);
}



void SubPowerNode::SetRkState(uint8_t rk1, uint8_t rk2)
{
    mrobot_driver_msgs::vci_can can_msg;
    CAN_ID_UNION id;
    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_READ_RK_STATE;
    id.CanID_Struct.SrcMACID = CAN_SUB_PB_SRC_ID;
    id.CanID_Struct.DestMACID = CAN_SUB_PB_DST_ID;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 0;
    id.CanID_Struct.res = 0;

//    ROS_INFO("Set RK State Build ID:0x%08x",id.CANx_ID);
    can_msg.ID = id.CANx_ID;
#if 1 
    can_msg.DataLen = 4;
    can_msg.Data.resize(4);
    can_msg.Data[0] = 0x00;
    can_msg.Data[1] = 2;
    can_msg.Data[2] = rk1;
    can_msg.Data[3] = rk2;
#endif
    this->sub_powerboard_pub.publish(can_msg);
}
void SubPowerNode::ReadVersionInfo(uint8_t type)
{
	mrobot_driver_msgs::vci_can can_msg;
	CAN_ID_UNION id;
    static uint32_t send_cnt = 0;
    send_cnt++;
    if((type == 0) || (type > 3))
    {
        ROS_INFO("ReadVersionInfo para ERROR!");
        return ;
    }
	memset(&id, 0x0, sizeof(CAN_ID_UNION));
	id.CanID_Struct.SourceID = CAN_SOURCE_ID_READ_VERSION;
	id.CanID_Struct.SrcMACID = CAN_SUB_PB_SRC_ID;
    id.CanID_Struct.DestMACID = CAN_SUB_PB_DST_ID;
	id.CanID_Struct.FUNC_ID = 0x02;
	id.CanID_Struct.ACK = 0;
	id.CanID_Struct.res = 0;
//	ROS_INFO("ReadVersion id: 0x%08x", id.CANx_ID);
	can_msg.ID = id.CANx_ID;
	can_msg.DataLen = 2;
	can_msg.Data.resize(2);
	can_msg.Data[0] = 0x00;
	can_msg.Data[1] = type;
	this->sub_powerboard_pub.publish(can_msg);
    ROS_INFO("send number:%d",send_cnt);
}

void SubPowerNode::ReadModuleState(uint8_t group_num)
{
    
    mrobot_driver_msgs::vci_can can_msg;
    CAN_ID_UNION id;
    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_MODULE_STATE;
    id.CanID_Struct.SrcMACID = CAN_SUB_PB_SRC_ID;
    id.CanID_Struct.DestMACID = CAN_SUB_PB_DST_ID;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 0;
    id.CanID_Struct.res = 0;

//    ROS_INFO("Read Module State id: 0x%08x", id.CANx_ID);
    can_msg.ID = id.CANx_ID;
    can_msg.DataLen = 8;
    can_msg.Data.resize(8);
    can_msg.Data[0] = 0x00;
    can_msg.Data[1] = 2;//read cmd
    can_msg.Data[2] = group_num;//1,2
    can_msg.Data[3] = 0xff;
    can_msg.Data[4] = 0xff;

    can_msg.Data[5] = 0xff;
    can_msg.Data[6] = 0xff;
    can_msg.Data[7] = 1;
    this->sub_powerboard_pub.publish(can_msg);
}

void SubPowerNode::SetModeluState(uint8_t group_num, uint32_t state, uint8_t on_off)
{
    mrobot_driver_msgs::vci_can can_msg;
    CAN_ID_UNION id;
    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_MODULE_STATE;
    id.CanID_Struct.SrcMACID = CAN_SUB_PB_SRC_ID;
    id.CanID_Struct.DestMACID = CAN_SUB_PB_DST_ID;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 0;
    id.CanID_Struct.res = 0;

//    ROS_INFO("Set Module State id: 0x%08x", id.CANx_ID);
    can_msg.ID = id.CANx_ID;
    can_msg.DataLen = 8;
    can_msg.Data.resize(8);
    can_msg.Data[0] = 0x00;
    can_msg.Data[1] = 1;//write cmd
    can_msg.Data[2] = group_num;//1,2
    can_msg.Data[3] = state>>24;
    can_msg.Data[4] = state>>16 & 0xff;
    can_msg.Data[5] = state>>8 & 0xff;
    can_msg.Data[6] = state & 0xff;
    can_msg.Data[7] = on_off;
    this->sub_powerboard_pub.publish(can_msg);
}

void SubPowerNode::TestFunc(void)
{
    mrobot_driver_msgs::vci_can can_msg;
    CAN_ID_UNION id;
    static uint8_t dest_id = 0;
    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_READ_RK_STATE;
    id.CanID_Struct.SrcMACID = CAN_SUB_PB_SRC_ID;
    id.CanID_Struct.DestMACID = dest_id++;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 0;
    id.CanID_Struct.res = 0;
    ROS_INFO("Test Func id: 0x%08x", id.CANx_ID);
    can_msg.ID = id.CANx_ID;
#if 0
   can_msg.DataLen = 4;
    can_msg.Data.resize(4);
    can_msg.Data[0] = 0;
    can_msg.Data[1] = 1;
    can_msg.Data[2] = 0;
    can_msg.Data[3] = 0;
#endif
#if 1 
#define TEST_LEN    50
    can_msg.DataLen = TEST_LEN;
        can_msg.Data.resize(TEST_LEN);
        for(uint8_t i = 0; i < TEST_LEN; i++)
        {
            can_msg.Data[i] = i;            
        }
#endif
   this->sub_powerboard_pub.publish(can_msg);
}
