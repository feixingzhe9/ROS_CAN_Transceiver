#include <ros/ros.h>
#include <std_msgs/String.h>
#include <mrobot_driver_msgs/vci_can.h>
#include "cm_can_id.h"
#include "can_powerboard.h"
#include <roscan/can_long_frame.h>
#include "json.hpp"

using json = nlohmann::json;
//using namespace nlohmann;

class PowerNode
{
    public:
        can_long_frame long_frame;

        PowerNode()
        {
            powerboard_pub = n.advertise<mrobot_driver_msgs::vci_can>("tx_power_node", 1000);
            powerboard_sub = n.subscribe("rx_power_node", 1000, &PowerNode::canReceivedCallback, this);
            to_app_pub = n.advertise<std_msgs::String>("/app_main_power_sub", 1000);
            from_app_sub= n.subscribe("/app_main_power_pub", 1000, &PowerNode::fromAppReceivedCallback, this);

            this->total_receive = 0;
        }
        void fromAppReceivedCallback(const std_msgs::String::ConstPtr &msg);
        void canReceivedCallback(const mrobot_driver_msgs::vci_can::ConstPtr& msg);
        void readSystemStatus( void );
        void ledsControl( uint8_t mode, uint16_t effect );
        void readBatteryVoltage( void );
        void readBatteryOutputCurrent( void );
        void readBatteryPercentage( void );
        void deviceManage( device_type_t device, device_contorl_t mode );
        void readFwVersion( void );
    private:
        ros::NodeHandle n;
        ros::Publisher powerboard_pub;
        ros::Subscriber powerboard_sub;
        ros::Publisher to_app_pub;
        ros::Subscriber from_app_sub;

        unsigned int total_receive;
        char fw_version[20];
        char hw_version[10];
        uint8_t     _system_status;
        uint8_t     _leds_mode;
        uint8_t     _leds_effect;
        uint16_t    _battery_voltage;
        int16_t     _battery_current;
        uint8_t     _battery_percentage;
        uint8_t     _battery_bits;
        typedef struct _module_state_t {
            uint8_t     power_state;
            uint16_t    current;
            uint8_t     is_abnormal;
        } module_state_t;
        module_state_t  _nv;
        module_state_t  _pc;
        module_state_t  _printer;
        module_state_t  _iccard;
        module_state_t  _5v_res;
        module_state_t  _12v_res;
        module_state_t  _24v_res;
        typedef struct _dcdc_module_state_t {
            uint8_t     power_state;
            uint16_t    current;
            uint8_t     is_abnormal;
            int8_t      temperature;
            uint8_t     is_temp_abnormal;
        } dcdc_module_state_t;
        dcdc_module_state_t     _5v_dcdc;
        dcdc_module_state_t     _12v_dcdc;
        dcdc_module_state_t     _24v_dcdc;
        void printInfo( void );
        json j;
        void publish_json_msg_to_app( const nlohmann::json j_msg);
};

const char sys_status_str[4][20] = {
    "power off","is power on","power on","is power off",
};
void PowerNode::printInfo( void )
{
#if 1
#if 0
    this->j = {
        {"fw_version", this->fw_version},
        {"sys_status", sys_status_str[this->_system_status] },
        {"leds_state", {
            {"mode", this->_leds_mode },
            {"effect",  this->_leds_effect }}
        },
        {"battery", {
            {"voltage_mv", this->_battery_voltage },
            {"current_ma", this->_battery_current },
            {"percentage", this->_battery_percentage}}
        },
        {"nv", {
            {"power_state", this->_nv.power_state },
            {"current_ma", this->_nv.current }, 
            {"is_abnormal", this->_nv.is_abnormal }}
        },
        {"pc", {
            {"power_state", this->_pc.power_state },
            {"current_ma", this->_pc.current }, 
            {"is_abnormal", this->_pc.is_abnormal }}
        },
        {"printer", {
            {"power_state", this->_printer.power_state },
            {"current_ma", this->_printer.current }, 
            {"is_abnormal", this->_printer.is_abnormal }}
        },
        {"iccard", {
            {"power_state", this->_iccard.power_state },
            {"current_ma", this->_iccard.current }, 
            {"is_abnormal", this->_iccard.is_abnormal }}
        },
        {"5v_res", {
            {"power_state", this->_5v_res.power_state },
            {"current_ma", this->_5v_res.current }, 
            {"is_abnormal", this->_5v_res.is_abnormal }}
        },
        {"12v_res", {
            {"power_state", this->_12v_res.power_state },
            {"current_ma", this->_12v_res.current }, 
            {"is_abnormal", this->_12v_res.is_abnormal }}
        },
        {"24v_res", {
            {"power_state", this->_24v_res.power_state },
            {"current_ma", this->_24v_res.current }, 
            {"is_abnormal", this->_24v_res.is_abnormal }}
        },
        {"5v_dcdc", {
            {"power_state", this->_5v_dcdc.power_state },
            {"current_ma", this->_5v_dcdc.current }, 
            {"is_abnormal", this->_5v_dcdc.is_abnormal },
            {"temperature_c", this->_5v_dcdc.temperature },
            {"is_temp_abnormal", this->_5v_dcdc.is_temp_abnormal }},
        },
        {"12v_dcdc", {
            {"power_state", this->_12v_dcdc.power_state },
            {"current_ma", this->_12v_dcdc.current }, 
            {"is_abnormal", this->_12v_dcdc.is_abnormal },
            {"temperature_c", this->_12v_dcdc.temperature },
            {"is_temp_abnormal", this->_12v_dcdc.is_temp_abnormal }},
        },
        {"24v_dcdc", {
            {"power_state", this->_24v_dcdc.power_state },
            {"current_ma", this->_24v_dcdc.current }, 
            {"is_abnormal", this->_24v_dcdc.is_abnormal },
            {"temperature_c", this->_24v_dcdc.temperature },
            {"is_temp_abnormal", this->_24v_dcdc.is_temp_abnormal }},
        }
    };
#endif
    //std::cout << j << std::endl;
    std::cout << std::setw(4) << std::setfill(' ') << j << std::endl;
#else
    printf("\n");
    printf("system status: %s\n", sys_status_str[this->_system_status]);
    printf("leds state: mode = 0x%02x, effect = 0x%04x\n", this->_leds_mode, this->_leds_effect);
    printf("battery voltage: %d mV\n", this->_battery_voltage);
    printf("battery current: %d mA\n", this->_battery_current);
    printf("battery percentage: %d%%\n", this->_battery_percentage);
    printf("nv device: power_state = %d, current = %d mA, is_abnormal = %d\n", \
                    this->_nv.power_state, this->_nv.current, this->_nv.is_abnormal);
    printf("pc device: power_state = %d, current = %d mA, is_abnormal = %d\n", \
                    this->_pc.power_state, this->_pc.current, this->_pc.is_abnormal);
    printf("printer device: power_state = %d, current = %d mA, is_abnormal = %d\n", \
                    this->_printer.power_state, this->_printer.current, this->_printer.is_abnormal);
    printf("iccard device: power_state = %d, current = %d mA, is_abnormal = %d\n", \
                    this->_iccard.power_state, this->_iccard.current, this->_iccard.is_abnormal);
    printf("5v_res device: power_state = %d, current = %d mA, is_abnormal = %d\n", \
                    this->_5v_res.power_state, this->_5v_res.current, this->_5v_res.is_abnormal);
    printf("12v_res device: power_state = %d, current = %d mA, is_abnormal = %d\n", \
                    this->_12v_res.power_state, this->_12v_res.current, this->_12v_res.is_abnormal);
    printf("24v_res device: power_state = %d, current = %d mA, is_abnormal = %d\n", \
                    this->_24v_res.power_state, this->_24v_res.current, this->_24v_res.is_abnormal);
    printf("5v_dcdc device: power_state = %d, current = %d mA, is_abnormal = %d, temperature = %d, is_temp_abnormal = %d\n",\
                this->_5v_dcdc.power_state, this->_5v_dcdc.current, this->_5v_dcdc.is_abnormal, this->_5v_dcdc.temperature, this->_5v_dcdc.is_temp_abnormal);
    printf("12v_dcdc device: power_state = %d, current = %d mA, is_abnormal = %d,temperature = %d, is_temp_abnormal = %d\n", \
                this->_12v_dcdc.power_state, this->_12v_dcdc.current, this->_12v_dcdc.is_abnormal, this->_12v_dcdc.temperature, this->_12v_dcdc.is_temp_abnormal);
    printf("24v_dcdc device: power_state = %d, current = %d mA, is_abnormal = %d, temperature = %d, is_temp_abnormal = %d\n", \
                this->_24v_dcdc.power_state, this->_24v_dcdc.current, this->_24v_dcdc.is_abnormal, this->_24v_dcdc.temperature, this->_24v_dcdc.is_temp_abnormal);
#endif
}

void PowerNode::publish_json_msg_to_app( const nlohmann::json j_msg)
{
    std_msgs::String pub_json_msg;
    std::stringstream ss;

    ss.clear();
    ss << j_msg;
    pub_json_msg.data = ss.str();
    this->to_app_pub.publish(pub_json_msg);
}

void PowerNode::fromAppReceivedCallback(const std_msgs::String::ConstPtr &msg)
{
    auto j = json::parse(msg->data.c_str());
    if( j.is_object() )
    {
        if ( j.find("pub_name") != j.end() )
        {
            if ( j["pub_name"] == "sys_status")
            {
                readSystemStatus();
            }
            else if ( j["pub_name"] == "sw_version")
            {
                readFwVersion();
            }
            else if ( j["pub_name"] == "battery")
            {
                _battery_bits = 0x00;
                readBatteryVoltage();
                readBatteryOutputCurrent();
                readBatteryPercentage();
            }
            else if ( j["pub_name"] == "nv")
            {
                deviceManage( DEVICE_TYPE_NV, DEVICE_CONTROL_MODE_READ );
            }
            else if ( j["pub_name"] == "pc")
            {
                deviceManage( DEVICE_TYPE_PC, DEVICE_CONTROL_MODE_READ );
            }
            else if ( j["pub_name"] == "printer")
            {
                deviceManage( DEVICE_TYPE_PRINTER, DEVICE_CONTROL_MODE_READ );
            }
            else if ( j["pub_name"] == "iccard")
            {
                deviceManage( DEVICE_TYPE_ICCARD, DEVICE_CONTROL_MODE_READ );
            }
            else if ( j["pub_name"] == "5v_res")
            {
                deviceManage( DEVICE_TYPE_5V_RES, DEVICE_CONTROL_MODE_READ );
            }
            else if ( j["pub_name"] == "12v_res")
            {
                deviceManage( DEVICE_TYPE_12V_RES, DEVICE_CONTROL_MODE_READ );
            }
            else if ( j["pub_name"] == "24v_res")
            {
                deviceManage( DEVICE_TYPE_24V_RES, DEVICE_CONTROL_MODE_READ );
            }
            else if ( j["pub_name"] == "5v_dcdc")
            {
                deviceManage( DEVICE_TYPE_5V_DCDC, DEVICE_CONTROL_MODE_READ );
            }
            else if ( j["pub_name"] == "12v_dcdc")
            {
                deviceManage( DEVICE_TYPE_12V_DCDC, DEVICE_CONTROL_MODE_READ );
            }
            else if ( j["pub_name"] == "24v_dcdc")
            {
                deviceManage( DEVICE_TYPE_24V_DCDC, DEVICE_CONTROL_MODE_READ );
            }
            else if ( j["pub_name"] == "leds_state")
            {
                if (j["data"].is_object())
                {
                    if ( j["data"]["mode"].is_number() && j["data"]["effect"].is_number() )
                    {
                        ledsControl(j["data"]["mode"], j["data"]["effect"]);
                    }
                }
            }
            else
            {
                std::string s = j["pub_name"];
                ROS_INFO("not found pub_name: %s", s.c_str());
            }
        }
        else
        {
            ROS_INFO("received msg is a not cantain \"pub_name\" json object");
        }
    }
    else
    {
        ROS_INFO("received msg is not a json object");
    }
    std::cout << std::setw(4) << std::setfill(' ') << j << std::endl;
}

void PowerNode::canReceivedCallback(const mrobot_driver_msgs::vci_can::ConstPtr& msg)
{
    can_id_union		can_id_u;
    mrobot_driver_msgs::vci_can long_msg;
    this->total_receive ++;
    can_id_u.can_id = msg->ID;
    if( can_id_u.can_id_stru.SrcMacID != PB_MASTER_NODE_MAC_ID )
    {
        ROS_INFO("this is not powerboard can msg");
        return;
    }
    if( can_id_u.can_id_stru.DstMacID != IPC_NODE_MAC_ID )
    {
        ROS_INFO("this is not send to ipc can msg");
        return;
    }
    //ROS_INFO("power_board received No.%d", this->total_receive);
    //ROS_INFO("msg.ID = 0x%08x", can_id_u.can_id);
    //ROS_INFO("msg.DataLen = %d", msg->DataLen);
    switch(can_id_u.can_id_stru.SourceID)
    {
        case SOURCE_ID_FW_VERSION:
            long_msg = long_frame.frame_construct(msg);
            if( long_msg.ID == 0 )
            {
                break;
            }
            //ROS_INFO("received fireware version");
            memcpy(this->fw_version, (const void *)&long_msg.Data[0], long_msg.DataLen);
            //ROS_INFO("fw_version:%s",this->fw_version);
            this->j.clear();
            this->j = {
                {"sub_name", "sw_version"},
                {"data", this->fw_version },
            };
            this->publish_json_msg_to_app(this->j);
            this->printInfo(); 
            break;
        case SOURCE_ID_SYSTEM_STATUS:
            this->_system_status = msg->Data[1];
            //ROS_INFO("received system status: 0x%02x", this->_system_status);
            this->j.clear();
            this->j = {
                {"sub_name", "sys_status"},
                {"data", sys_status_str[this->_system_status]},
            };
            this->publish_json_msg_to_app(this->j);
            this->printInfo(); 
            break;
        case SOURCE_ID_LEDS:
            this->_leds_mode = msg->Data[1];
            this->_leds_effect = msg->Data[2] << 8 | msg->Data[3];
            //ROS_INFO("received leds state: mode = 0x%02x, effect = 0x%04x", this->_leds_mode, this->_leds_effect);
            this->j.clear();
            this->j = {
                {"sub_name", "leds_state"},
                {"data",{ 
                            {"mode", this->_leds_mode },
                            {"effect",  this->_leds_effect },
                        }
                }
            };
            this->publish_json_msg_to_app(this->j);
            this->printInfo(); 
            break;
        case SOURCE_ID_BATT_VOLTAGE:
            this->_battery_voltage = msg->Data[1] << 8 | msg->Data[2];
            //ROS_INFO("received battery voltage: %d mV", this->_battery_voltage);
            _battery_bits |= 0x01;
            if( _battery_bits != 0x07)
                break;

            this->j.clear();
            this->j = {
                {"sub_name", "battery"},
                {"data",{ 
                            {"voltage_mv", this->_battery_voltage },
                            {"current_ma", this->_battery_current },
                            {"percentage", this->_battery_percentage},
                        }
                }
            };
            this->publish_json_msg_to_app(this->j);
            this->printInfo(); 
            break;
        case SOURCE_ID_BATT_CURRENT:
            this->_battery_current = msg->Data[1] << 8 | msg->Data[2];
            //ROS_INFO("received battery current: %d mA", this->_battery_current);
            _battery_bits |= 0x02;
            if( _battery_bits != 0x07)
                break;

            this->j.clear();
            this->j = {
                {"sub_name", "battery"},
                {"data",{ 
                            {"voltage_mv", this->_battery_voltage },
                            {"current_ma", this->_battery_current },
                            {"percentage", this->_battery_percentage},
                        }
                }
            };
            this->publish_json_msg_to_app(this->j);
            this->printInfo(); 
            break;
        case SOURCE_ID_BATT_PERCENTAGE:
            this->_battery_percentage = msg->Data[1];
            //ROS_INFO("received battery percentage: %d%%", this->_battery_percentage);
            _battery_bits |= 0x04;
            if( _battery_bits != 0x07)
                break;

            this->j.clear();
            this->j = {
                {"sub_name", "battery"},
                {"data",{ 
                            {"voltage_mv", this->_battery_voltage },
                            {"current_ma", this->_battery_current },
                            {"percentage", this->_battery_percentage},
                        }
                }
            };
            this->publish_json_msg_to_app(this->j);
            this->printInfo(); 
            break;
        case SOURCE_ID_NV:
            this->_nv.power_state = msg->Data[1];
            this->_nv.current = (uint16_t)msg->Data[2] << 8 | msg->Data[3];
            this->_nv.is_abnormal = msg->Data[4];
            //ROS_INFO("received nv device: power_state = %d, current = %d mA, is_abnormal = %d", \
                    this->_nv.power_state, this->_nv.current, this->_nv.is_abnormal);
            this->j.clear();
            this->j = {
                {"sub_name", "nv"},
                {"data",{ 
                            {"power_state", bool(this->_nv.power_state) },
                            {"current_ma", this->_nv.current }, 
                            {"is_abnormal", bool(this->_nv.is_abnormal) },
                        }
                }
            };
            this->publish_json_msg_to_app(this->j);
            this->printInfo(); 
            break;
        case SOURCE_ID_PC:
            this->_pc.power_state = msg->Data[1];
            this->_pc.current = (uint16_t)msg->Data[2] << 8 | msg->Data[3];
            this->_pc.is_abnormal = msg->Data[4];
            //ROS_INFO("received pc device: power_state = %d, current = %d mA, is_abnormal = %d", \
                    this->_pc.power_state, this->_pc.current, this->_pc.is_abnormal);
            this->j.clear();
            this->j = {
                {"sub_name", "pc"},
                {"data",{ 
                            {"power_state", bool(this->_pc.power_state) },
                            {"current_ma", this->_pc.current }, 
                            {"is_abnormal", bool(this->_pc.is_abnormal) }
                        }
                }
            };
            this->publish_json_msg_to_app(this->j);
            this->printInfo(); 
            break;
        case SOURCE_ID_PRINTER:
            this->_printer.power_state = msg->Data[1];
            this->_printer.current = (uint16_t)msg->Data[2] << 8 | msg->Data[3];
            this->_printer.is_abnormal = msg->Data[4];
            //ROS_INFO("received printer device: power_state = %d, current = %d mA, is_abnormal = %d", \
                    this->_printer.power_state, this->_printer.current, this->_printer.is_abnormal);
            this->j.clear();
            this->j = {
                {"sub_name", "printer"},
                {"data",{
                            {"power_state", bool(this->_printer.power_state) },
                            {"current_ma", this->_printer.current }, 
                            {"is_abnormal", bool(this->_printer.is_abnormal) }
                        }
                }
            };
            this->publish_json_msg_to_app(this->j);
            this->printInfo(); 
            break;
        case SOURCE_ID_ICCARD:
            this->_iccard.power_state = msg->Data[1];
            this->_iccard.current = (uint16_t)msg->Data[2] << 8 | msg->Data[3];
            this->_iccard.is_abnormal = msg->Data[4];
            //ROS_INFO("received iccard device: power_state = %d, current = %d mA, is_abnormal = %d", \
                    this->_iccard.power_state, this->_iccard.current, this->_iccard.is_abnormal);
            this->j.clear();
            this->j = {
                {"sub_name", "iccard"},
                {"data",{
                            {"power_state", bool(this->_iccard.power_state) },
                            {"current_ma", this->_iccard.current }, 
                            {"is_abnormal", bool(this->_iccard.is_abnormal) }
                        }
                }
            };
            this->publish_json_msg_to_app(this->j);
            this->printInfo(); 
            break;
        case SOURCE_ID_5V_RES:
            this->_5v_res.power_state = msg->Data[1];
            this->_5v_res.current = (uint16_t)msg->Data[2] << 8 | msg->Data[3];
            this->_5v_res.is_abnormal = msg->Data[4];
            //ROS_INFO("received 5v_res device: power_state = %d, current = %d mA, is_abnormal = %d", \
                    this->_5v_res.power_state, this->_5v_res.current, this->_5v_res.is_abnormal);
            this->j.clear();
            this->j = {
                {"sub_name", "5v_res"},
                {"data",{
                            {"power_state", bool(this->_5v_res.power_state) },
                            {"current_ma", this->_5v_res.current }, 
                            {"is_abnormal", bool(this->_5v_res.is_abnormal) }
                        }
                }
            };
            this->publish_json_msg_to_app(this->j);
            this->printInfo(); 
            break;
        case SOURCE_ID_12V_RES:
            this->_12v_res.power_state = msg->Data[1];
            this->_12v_res.current = (uint16_t)msg->Data[2] << 8 | msg->Data[3];
            this->_12v_res.is_abnormal = msg->Data[4];
            //ROS_INFO("received 12v_res device: power_state = %d, current = %d mA, is_abnormal = %d", \
                    this->_12v_res.power_state, this->_12v_res.current, this->_12v_res.is_abnormal);
            this->j.clear();
            this->j = {
                {"sub_name", "12v_res"},
                {"data",{
                            {"power_state", bool(this->_12v_res.power_state) },
                            {"current_ma", this->_12v_res.current }, 
                            {"is_abnormal", bool(this->_12v_res.is_abnormal) }
                        }
                }
            };
            this->publish_json_msg_to_app(this->j);
            this->printInfo(); 
            break;
        case SOURCE_ID_24V_RES:
            this->_24v_res.power_state = msg->Data[1];
            this->_24v_res.current = (uint16_t)msg->Data[2] << 8 | msg->Data[3];
            this->_24v_res.is_abnormal = msg->Data[4];
            //ROS_INFO("received 24v_res device: power_state = %d, current = %d mA, is_abnormal = %d", \
                    this->_24v_res.power_state, this->_24v_res.current, this->_24v_res.is_abnormal);
            this->j.clear();
            this->j = {
                {"sub_name", "24v_res"},
                {"data",{
                            {"power_state", bool(this->_24v_res.power_state) },
                            {"current_ma", this->_24v_res.current }, 
                            {"is_abnormal", bool(this->_24v_res.is_abnormal) }
                        }
                }
            };
            this->publish_json_msg_to_app(this->j);
            this->printInfo(); 
            break;
        case SOURCE_ID_5V_DCDC:
            this->_5v_dcdc.power_state = msg->Data[1];
            this->_5v_dcdc.current = (uint16_t)msg->Data[2] << 8 | msg->Data[3];
            this->_5v_dcdc.is_abnormal = msg->Data[4];
            this->_5v_dcdc.temperature = msg->Data[5];
            this->_5v_dcdc.is_temp_abnormal = msg->Data[6];
            //ROS_INFO("received 5v_dcdc device: power_state = %d, current = %d mA, is_abnormal = %d, temperature = %d", \
                this->_5v_dcdc.power_state, this->_5v_dcdc.current, this->_5v_dcdc.is_abnormal, this->_5v_dcdc.temperature);
            this->j.clear();
            this->j = {
                {"sub_name", "5v_dcdc"},
                {"data",{
                            {"power_state", bool(this->_5v_dcdc.power_state) },
                            {"current_ma", this->_5v_dcdc.current }, 
                            {"is_abnormal", bool(this->_5v_dcdc.is_abnormal) },
                            {"temperature_c", this->_5v_dcdc.temperature },
                            {"is_temp_abnormal", bool(this->_5v_dcdc.is_temp_abnormal) }
                        }
                }
            };
            this->publish_json_msg_to_app(this->j);
            this->printInfo(); 
            break;
        case SOURCE_ID_12V_DCDC:
            this->_12v_dcdc.power_state = msg->Data[1];
            this->_12v_dcdc.current = (uint16_t)msg->Data[2] << 8 | msg->Data[3];
            this->_12v_dcdc.is_abnormal = msg->Data[4];
            this->_12v_dcdc.temperature = msg->Data[5];
            this->_5v_dcdc.is_temp_abnormal = msg->Data[6];
            //ROS_INFO("received 12v_dcdc device: power_state = %d, current = %d mA, is_abnormal = %d,temperature = %d", \
                this->_12v_dcdc.power_state, this->_12v_dcdc.current, this->_12v_dcdc.is_abnormal, this->_12v_dcdc.temperature);
            this->j.clear();
            this->j = {
                {"sub_name", "12v_dcdc"},
                {"data",{
                            {"power_state", bool(this->_12v_dcdc.power_state) },
                            {"current_ma", this->_12v_dcdc.current }, 
                            {"is_abnormal", bool(this->_12v_dcdc.is_abnormal) },
                            {"temperature_c", this->_12v_dcdc.temperature },
                            {"is_temp_abnormal", bool(this->_12v_dcdc.is_temp_abnormal) }
                        }
                }
            };
            this->publish_json_msg_to_app(this->j);
            this->printInfo(); 
            break;
        case SOURCE_ID_24V_DCDC:
            this->_24v_dcdc.power_state = msg->Data[1];
            this->_24v_dcdc.current = (uint16_t)msg->Data[2] << 8 | msg->Data[3];
            this->_24v_dcdc.is_abnormal = msg->Data[4];
            this->_24v_dcdc.temperature = msg->Data[5];
            this->_5v_dcdc.is_temp_abnormal = msg->Data[6];
            //ROS_INFO("received 24v_dcdc device: power_state = %d, current = %d mA, is_abnormal = %d, temperature = %d", \
                this->_24v_dcdc.power_state, this->_24v_dcdc.current, this->_24v_dcdc.is_abnormal, this->_24v_dcdc.temperature);
            this->j.clear();
            this->j = {
                {"sub_name", "24v_dcdc"},
                {"data",{
                            {"power_state", bool(this->_24v_dcdc.power_state) },
                            {"current_ma", this->_24v_dcdc.current }, 
                            {"is_abnormal", bool(this->_24v_dcdc.is_abnormal) },
                            {"temperature_c", this->_24v_dcdc.temperature },
                            {"is_temp_abnormal", bool(this->_24v_dcdc.is_temp_abnormal) }
                        }
                }
            };
            this->publish_json_msg_to_app(this->j);
            this->printInfo(); 
            break;
        default:
            ROS_INFO("received unknown message");
            break;
    }
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "powerboard_node");

    PowerNode powerNode;

    uint32_t count = 20;

    ros::Rate loop_rate(10);

    int flag = 1;
    sleep(1);

    uint8_t leds_mode = 0;
    uint8_t leds_effect = 0;
    while (ros::ok())
    {
        //powerNode.readFwVersion();
#if 0
        if( count == 21 )
        { 
            leds_effect++;
            if(leds_effect >= 16)
            {
                leds_effect = 0;
            }
            powerNode.ledsControl(0x09, leds_effect);
        }
        if( count == 22)
        {
            powerNode.readSystemStatus();
        }
        if( count == 23 )
        {
            powerNode.readBatteryVoltage();
        }
        if( count == 24 )
        {
            powerNode.readBatteryOutputCurrent();
        }
        if( count == 25 )
        {
            powerNode.readBatteryPercentage();
        }
        if( count == 26 )
        {
            powerNode.deviceManage( DEVICE_TYPE_NV, DEVICE_CONTROL_MODE_READ );
        }
        if( count == 27 )
        {
            powerNode.deviceManage( DEVICE_TYPE_PC, DEVICE_CONTROL_MODE_READ );
        }
        if( count == 28 )
        {
            powerNode.deviceManage( DEVICE_TYPE_PRINTER, DEVICE_CONTROL_MODE_READ );
        }
        if( count == 29 )
        {
            powerNode.deviceManage( DEVICE_TYPE_ICCARD, DEVICE_CONTROL_MODE_READ );
        }
        if( count == 30 )
        {
            powerNode.deviceManage( DEVICE_TYPE_5V_RES, DEVICE_CONTROL_MODE_READ );
        }
        if( count == 31 )
        {
            powerNode.deviceManage( DEVICE_TYPE_12V_RES, DEVICE_CONTROL_MODE_READ );
        }
        if( count == 32 )
        {
            powerNode.deviceManage( DEVICE_TYPE_24V_RES, DEVICE_CONTROL_MODE_READ );
        }
        if( count == 33 )
        {
            powerNode.deviceManage( DEVICE_TYPE_5V_DCDC, DEVICE_CONTROL_MODE_READ );
        }
        if( count == 34 )
        {
            powerNode.deviceManage( DEVICE_TYPE_12V_DCDC, DEVICE_CONTROL_MODE_READ );
        }
        if( count == 35 )
        {
            powerNode.deviceManage( DEVICE_TYPE_24V_DCDC, DEVICE_CONTROL_MODE_READ );
            count = 20;
        }
#endif
        if( flag )
        {
            ROS_INFO("send 1 time");
            flag = 0;
           // powerNode.ledsControl(0x09, 0x0003);
            powerNode.readFwVersion();
        }

        count ++;
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("powerboard node exit");

    return 0;
}

void PowerNode::readFwVersion( void )
{
   can_id_union		can_id_u;
   mrobot_driver_msgs::vci_can can_msg;


   can_id_u.can_id_stru.Reserve             = 0;
   can_id_u.can_id_stru.SrcMacID			= IPC_NODE_MAC_ID;
   can_id_u.can_id_stru.DstMacID			= PB_MASTER_NODE_MAC_ID;
   can_id_u.can_id_stru.Ack	    			= 0;
   can_id_u.can_id_stru.FuncID				= FUNC_ID_READ;
   can_id_u.can_id_stru.SourceID			= SOURCE_ID_FW_VERSION;

   can_msg.ID = can_id_u.can_id;
   ROS_DEBUG("build can.ID = 0x%08x", can_msg.ID);
   can_msg.DataLen = 1;
   can_msg.Data.resize(1);
   can_msg.Data[0]  = 0x00;

   ROS_DEBUG("read firware version");
   this->powerboard_pub.publish(can_msg);
}
void PowerNode::readSystemStatus( void )
{
   can_id_union		can_id_u;
   mrobot_driver_msgs::vci_can can_msg;

   can_id_u.can_id_stru.Reserve             = 0;
   can_id_u.can_id_stru.SrcMacID			= IPC_NODE_MAC_ID;
   can_id_u.can_id_stru.DstMacID			= PB_MASTER_NODE_MAC_ID;
   can_id_u.can_id_stru.Ack			     	= 0;
   can_id_u.can_id_stru.FuncID				= FUNC_ID_READ;
   can_id_u.can_id_stru.SourceID			= SOURCE_ID_SYSTEM_STATUS;

   can_msg.ID = can_id_u.can_id;
   ROS_DEBUG("build can.ID = 0x%08x", can_msg.ID);
   can_msg.DataLen = 1;
   can_msg.Data.resize(1);
   can_msg.Data[0]  = 0x00;

   ROS_DEBUG("read system status");
   this->powerboard_pub.publish(can_msg);
}

void PowerNode::ledsControl( uint8_t mode, uint16_t effect )
{
   can_id_union		can_id_u;
   mrobot_driver_msgs::vci_can can_msg;

   can_id_u.can_id_stru.Reserve             = 0;
   can_id_u.can_id_stru.SrcMacID			= IPC_NODE_MAC_ID;
   can_id_u.can_id_stru.DstMacID			= PB_MASTER_NODE_MAC_ID;
   can_id_u.can_id_stru.Ack			    	= 0;
   can_id_u.can_id_stru.FuncID				= FUNC_ID_WRITE;
   can_id_u.can_id_stru.SourceID			= SOURCE_ID_LEDS;

   can_msg.ID = can_id_u.can_id;
   ROS_DEBUG("build can.ID = 0x%08x", can_msg.ID);
   can_msg.DataLen = 4;
   can_msg.Data.resize(4);
   can_msg.Data[0]  = 0x00;
   can_msg.Data[1]  = mode;
   can_msg.Data[2]  = ( effect & 0xff00 ) >> 8;
   can_msg.Data[3]  = effect & 0x00ff;

   ROS_DEBUG("leds control mode:0x%02x, effect:0x%04x",mode, effect);
   this->powerboard_pub.publish(can_msg);
}

void PowerNode::readBatteryVoltage( void )
{
   can_id_union		can_id_u;
   mrobot_driver_msgs::vci_can can_msg;

   can_id_u.can_id_stru.Reserve             = 0;
   can_id_u.can_id_stru.SrcMacID			= IPC_NODE_MAC_ID;
   can_id_u.can_id_stru.DstMacID			= PB_MASTER_NODE_MAC_ID;
   can_id_u.can_id_stru.Ack			    	= 0;
   can_id_u.can_id_stru.FuncID				= FUNC_ID_READ;
   can_id_u.can_id_stru.SourceID			= SOURCE_ID_BATT_VOLTAGE;

   can_msg.ID = can_id_u.can_id;
   ROS_DEBUG("build can.ID = 0x%08x", can_msg.ID);
   can_msg.DataLen = 1;
   can_msg.Data.resize(1);
   can_msg.Data[0]  = 0x00;

   ROS_DEBUG("read battery voltage");
   this->powerboard_pub.publish(can_msg);
}

void PowerNode::readBatteryOutputCurrent( void )
{
   can_id_union		can_id_u;
   mrobot_driver_msgs::vci_can can_msg;

   can_id_u.can_id_stru.Reserve             = 0;
   can_id_u.can_id_stru.SrcMacID			= IPC_NODE_MAC_ID;
   can_id_u.can_id_stru.DstMacID			= PB_MASTER_NODE_MAC_ID;
   can_id_u.can_id_stru.Ack			    	= 0;
   can_id_u.can_id_stru.FuncID				= FUNC_ID_READ;
   can_id_u.can_id_stru.SourceID			= SOURCE_ID_BATT_CURRENT;

   can_msg.ID = can_id_u.can_id;
   ROS_DEBUG("build can.ID = 0x%08x", can_msg.ID);
   can_msg.DataLen = 1;
   can_msg.Data.resize(1);
   can_msg.Data[0]  = 0x00;

   ROS_DEBUG("read battery output current");
   this->powerboard_pub.publish(can_msg);
}

void PowerNode::readBatteryPercentage( void )
{
   can_id_union		can_id_u;
   mrobot_driver_msgs::vci_can can_msg;

   can_id_u.can_id_stru.Reserve             = 0;
   can_id_u.can_id_stru.SrcMacID			= IPC_NODE_MAC_ID;
   can_id_u.can_id_stru.DstMacID			= PB_MASTER_NODE_MAC_ID;
   can_id_u.can_id_stru.Ack			    	= 0;
   can_id_u.can_id_stru.FuncID				= FUNC_ID_READ;
   can_id_u.can_id_stru.SourceID			= SOURCE_ID_BATT_PERCENTAGE;

   can_msg.ID = can_id_u.can_id;
   ROS_DEBUG("build can.ID = 0x%08x", can_msg.ID);
   can_msg.DataLen = 1;
   can_msg.Data.resize(1);
   can_msg.Data[0]  = 0x00;

   ROS_DEBUG("read battery percentage");
   this->powerboard_pub.publish(can_msg);
}

/*
para: mode
    - mode = 0x00: read information of device
    - mode = 0x01: open device
    - mode = 0x02: close  device
   */
void PowerNode::deviceManage( device_type_t device, device_contorl_t mode )
{
   can_id_union		can_id_u;
   mrobot_driver_msgs::vci_can can_msg;

   can_id_u.can_id_stru.Reserve             = 0;
   can_id_u.can_id_stru.SrcMacID			= IPC_NODE_MAC_ID;
   can_id_u.can_id_stru.DstMacID			= PB_MASTER_NODE_MAC_ID;
   can_id_u.can_id_stru.Ack			    	= 0;

   switch( device )
   {
       case DEVICE_TYPE_NV:
           can_id_u.can_id_stru.SourceID = SOURCE_ID_NV;
           break;
       case DEVICE_TYPE_PC:
           can_id_u.can_id_stru.SourceID = SOURCE_ID_PC;
           break;
       case DEVICE_TYPE_PRINTER:
           can_id_u.can_id_stru.SourceID = SOURCE_ID_PRINTER;
           break;
       case DEVICE_TYPE_ICCARD:
           can_id_u.can_id_stru.SourceID = SOURCE_ID_ICCARD;
           break;
       case DEVICE_TYPE_5V_RES:
           can_id_u.can_id_stru.SourceID = SOURCE_ID_5V_RES;
           break;
       case DEVICE_TYPE_12V_RES:
           can_id_u.can_id_stru.SourceID = SOURCE_ID_12V_RES;
           break;
       case DEVICE_TYPE_24V_RES:
           can_id_u.can_id_stru.SourceID = SOURCE_ID_24V_RES;
           break;
       case DEVICE_TYPE_5V_DCDC:
           can_id_u.can_id_stru.SourceID = SOURCE_ID_5V_DCDC;
           break;
       case DEVICE_TYPE_12V_DCDC:
           can_id_u.can_id_stru.SourceID = SOURCE_ID_12V_DCDC;
           break;
       case DEVICE_TYPE_24V_DCDC:
           can_id_u.can_id_stru.SourceID = SOURCE_ID_24V_DCDC;
           break;
   }

   if( DEVICE_CONTROL_MODE_READ == mode )
   {
       can_id_u.can_id_stru.FuncID = FUNC_ID_READ;
       can_msg.ID = can_id_u.can_id;
       ROS_DEBUG("build can.ID = 0x%08x", can_msg.ID);
       can_msg.DataLen = 1;
       can_msg.Data.resize(1);
       can_msg.Data[0]  = 0x00;
       ROS_DEBUG("read device:%d information", device);
   }
   else
   {
       can_id_u.can_id_stru.FuncID = FUNC_ID_WRITE;
       can_msg.ID = can_id_u.can_id;
       ROS_DEBUG("build can.ID = 0x%08x", can_msg.ID);
       can_msg.DataLen = 2;
       can_msg.Data.resize(2);
       can_msg.Data[0] = 0x00;
       if( DEVICE_CONTROL_MODE_OPEN == mode )
       {
           can_msg.Data[1] = 0x01;
           ROS_DEBUG("open device:%d", device);
       }
       else if( DEVICE_CONTROL_MODE_CLOSE == mode )
       {
           can_msg.Data[1] = 0x00;
           ROS_DEBUG("close device:%d", device);
       }
   }

   this->powerboard_pub.publish(can_msg);
}

