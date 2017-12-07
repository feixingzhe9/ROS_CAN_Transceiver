#ifndef _CAN_SUB_PB_H
#define _CAN_SUB_PB_H

#define CAN_SUB_PB_SRC_ID   0
#define CAN_SUB_PB_DST_ID   0x51

//////  function id define  //////
#define CAN_FUN_ID_RESET        0x06
#define CAN_FUN_ID_WRITE        0x01
#define CAN_FUN_ID_READ         0x02
#define CAN_FUN_ID_TRIGGER      0x03


//////  source id define  //////
#define CAN_SOURCE_ID_READ_VERSION      0x01
#define CAN_SOURCE_ID_MODULE_STATE      0x81
#define CAN_SOURCE_ID_ERROR_STATE       0x82
#define CAN_SOURCE_ID_READ_ADC_DATA     0x83
#define CAN_SOURCE_ID_READ_RK_STATE     0x84
#define CAN_SOURCE_ID_PWM_LED           0x85   

typedef struct _VoltageData_t 
{
  uint16_t              c_5V_right_hand_motor;
  uint16_t              c_24V_hd_camera;
  uint16_t              c_24V_head_motor;
  uint16_t              c_24V_camera;
  uint16_t              c_5V_keyboard;
  uint16_t              c_5V_code;
  uint16_t              c_5V_left_hand_motor;
  uint16_t              c_5V_card_reader;
  uint16_t              c_5V_head;
  uint16_t              c_24V_right_hand_motor;
  uint16_t              c_12V_head_rk;
  uint16_t              c_12V_chest_rk;
  uint16_t              c_24V_left_hand_motor;
  uint16_t              c_12V_audio_pa;
  uint16_t              c_5V_repair;
  uint16_t              c_5V_touch;

  //uint16_t              c_switch;
  uint16_t              c_12V_router;
  uint16_t              c_vbus;
  uint16_t              c_12V_card_reader;
  uint16_t              c_12V_switch;
  
  uint16_t              temp_12V_ts;
  uint16_t              temp_5V_ts;
  uint16_t              temp_air_ts;
  uint16_t              c_5V_head_touch;
  uint16_t              c_12V_all;
  uint16_t              c_5V_all;
  uint16_t              c_5V_head_led;  //头部灯板
  uint16_t              v_12V;
  int16_t               v_5V;
  int16_t               v_bat;
  int16_t               c_5V_reserve;
  int16_t               c_5V_led;   //高拍 LED
  int16_t               c_5V_camera;
  int16_t               c_5V_hd_camera;
} voltageData_t;

typedef enum
{
#if 1
    POWER_HEAD_RK  = 0x01,
    POWER_CHEST_RK = 0x02,
    POWER_ALL_RK   = 0x0F
#endif
} PowerControl_TypeDef;

enum module {
    dcdc_12v                = 0x00000001,
    dcdc_5v                 = 0x00000002,
    rk_head                 = 0x00000004,
    rk_head_res             = 0x00000008,
    rk_chest                = 0x00000010,
    rk_chest_res            = 0x00000020,
    left_hand_motor_24v     = 0x00000040,
    right_hand_motor_24v    = 0x00000080,
    head_motor_24v          = 0x00000100,
    switch_12v              = 0x00000200,
    router_12v              = 0x00000400,
    card_reader_12v         = 0x00000800,
    audio_pa_12v            = 0x00001000,
    left_hand_motor_5v      = 0x00002000,
    right_hand_motor_5v     = 0x00004000,
    head_motor_5v           = 0x00008000,
    reserve_5v              = 0x00010000,
    led_boards_5v           = 0x00020000,
    repair_board_5v         = 0x00040000,
    touch_board_5v          = 0x00080000,
    code_board_5v           = 0x00100000,
    card_reader_5v          = 0x00200000,
    bar_code_reader_5v      = 0x00400000,
    carmera_5v              = 0x00800000,
    hd_carmera_5v           = 0x01000000,
    head_led_5v             = 0x02000000,
    hd_carmera_24v          = 0x04000000,
    carmera_24v             = 0x08000000,
    head_touch_5v           = 0x10000000,
};

enum module_group2 {
    led_card_finger         = 0x00000001,
    led_a4_printer          = 0x00000002,
    led_ticket_printer      = 0x00000004,
    led_card_machine        = 0x00000008,
};
#endif
