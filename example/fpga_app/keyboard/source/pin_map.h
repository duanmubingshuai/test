#ifndef __PIN_MAP_H__
#define __PIN_MAP_H__

#include "gpio.h"
#define PHY6230_REMOTE_CONTROL_VERSION
//#define PHY6252_REMOTE_CONTROL_VERSION
//#define PHY6222_REMOTE_CONTROL_VERSION



typedef enum
{
    KEY_ROW_P00   =   0,
    KEY_ROW_P01   =   1,
    KEY_ROW_P02   =   2,
    KEY_ROW_P03   =   3,
    KEY_ROW_P04   =   4,
} KSCAN_ROWS_e;

typedef enum
{
    KEY_COL_P00   =   0,
    KEY_COL_P01   =   1,
    KEY_COL_P02   =   2,
    KEY_COL_P03   =   3,
    KEY_COL_P04   =   4,
} KSCAN_COLS_e;


#ifdef PHY6230_REMOTE_CONTROL_VERSION
// !infrared learn control gpio config


// !row gpio
#define KSCAN_ROW_0_GPIO			  GPIO_P00
#define KSCAN_ROW_1_GPIO              GPIO_P02
#define KSCAN_ROW_2_GPIO              GPIO_P07
#define KSCAN_ROW_3_GPIO              GPIO_P08

// !col gpio
#define KSCAN_COL_0_GPIO			  GPIO_P11
#define KSCAN_COL_1_GPIO              GPIO_P03
#define KSCAN_COL_2_GPIO              GPIO_P14
#define KSCAN_COL_3_GPIO              GPIO_P17//GPIO_P24//GPIO_P17

// !row col map kscan driver value
#define KSCAN_MAP_GPIO_ROW0           KEY_ROW_P00
#define KSCAN_MAP_GPIO_ROW1           KEY_ROW_P01
#define KSCAN_MAP_GPIO_ROW2           KEY_ROW_P02
#define KSCAN_MAP_GPIO_ROW3           KEY_ROW_P03
#define KSCAN_MAP_GPIO_COL0           KEY_COL_P00
#define KSCAN_MAP_GPIO_COL1           KEY_COL_P01
#define KSCAN_MAP_GPIO_COL2           KEY_COL_P02
#define KSCAN_MAP_GPIO_COL3           KEY_COL_P03//KEY_COL_P24//KEY_COL_P17

#endif



#ifdef PHY6252_REMOTE_CONTROL_VERSION
// !infrared learn control gpio config
#define IR_LEARAN_CTL_GPIO            GPIO_P33

// !infrared sending gpio
#define IR_SENDING_GPIO               GPIO_P31

// !infrared led display gpio
#define IR_DISPLAY_GPIO               GPIO_P24 //GPIO_P32//GPIO_P24

// !infrared led learn input gpio
#define IR_LEARN_GPIO                 GPIO_P32

// ![amic]pga_in+ gpio
#define VOICE_PGA_IN_ADD_GPIO         GPIO_P18

// ![amic]pga_in- gpio
#define VOICE_PGA_IN_REDUCE_GPIO      GPIO_P20

// ![amic]micphone bias gpio
#define VOICE_MIC_BIAS_GPIO           GPIO_P15

#define VOICE_MIC_REF_GPIO            GPIO_P23

// ![dmic]clk_gpio
#define VOICE_CLK_GPIO                GPIO_P15

// ![dmic]data_gpio
#define VOICE_DATA_GPIO               GPIO_P14

// !row gpio
#define KSCAN_ROW_0_GPIO			  GPIO_P00
#define KSCAN_ROW_1_GPIO              GPIO_P02
#define KSCAN_ROW_2_GPIO              GPIO_P07
#define KSCAN_ROW_3_GPIO              GPIO_P08

// !col gpio
#define KSCAN_COL_0_GPIO			  GPIO_P11
#define KSCAN_COL_1_GPIO              GPIO_P03
#define KSCAN_COL_2_GPIO              GPIO_P14
#define KSCAN_COL_3_GPIO              GPIO_P17//GPIO_P24//GPIO_P17

// !row col map kscan driver value
#define KSCAN_MAP_GPIO_ROW0           KEY_ROW_P00
#define KSCAN_MAP_GPIO_ROW1           KEY_ROW_P02
#define KSCAN_MAP_GPIO_ROW2           KEY_ROW_P07
#define KSCAN_MAP_GPIO_ROW3           KEY_ROW_P34
#define KSCAN_MAP_GPIO_COL0           KEY_COL_P11
#define KSCAN_MAP_GPIO_COL1           KEY_COL_P03
#define KSCAN_MAP_GPIO_COL2           KEY_COL_P14
#define KSCAN_MAP_GPIO_COL3           KEY_COL_P17//KEY_COL_P24//KEY_COL_P17

#endif

#ifdef PHY6222_REMOTE_CONTROL_VERSION
// !infrared learn control gpio config
#define IR_LEARAN_CTL_GPIO            GPIO_P33

// !infrared sending gpio
#define IR_SENDING_GPIO               GPIO_P31

// !infrared led display gpio
#define IR_DISPLAY_GPIO               GPIO_P16

// !infrared led learn input gpio
#define IR_LEARN_GPIO                 GPIO_P32

// ![amic]pga_in+ gpio
#define VOICE_PGA_IN_ADD_GPIO         GPIO_P18

// ![amic]pga_in- gpio
#define VOICE_PGA_IN_REDUCE_GPIO      GPIO_P20

// ![amic]micphone bias gpio
#define VOICE_MIC_BIAS_GPIO           GPIO_P15

// ![dmic]clk_gpio
#define VOICE_CLK_GPIO                GPIO_P15

// ![dmic]data_gpio
#define VOICE_DATA_GPIO               GPIO_P14

// !row gpio
#define KSCAN_ROW_0_GPIO			  GPIO_P00
#define KSCAN_ROW_1_GPIO              GPIO_P02
#define KSCAN_ROW_2_GPIO              GPIO_P07
#define KSCAN_ROW_3_GPIO              GPIO_P34
#define KSCAN_ROW_4_GPIO              GPIO_P25

// !col gpio
#define KSCAN_COL_0_GPIO			  GPIO_P01
#define KSCAN_COL_1_GPIO              GPIO_P03
#define KSCAN_COL_2_GPIO              GPIO_P11
#define KSCAN_COL_3_GPIO              GPIO_P14
#define KSCAN_COL_4_GPIO              GPIO_P24
#define KSCAN_COL_5_GPIO              GPIO_P26

// !row col map kscan driver value
#define KSCAN_MAP_GPIO_ROW0           KEY_ROW_P00
#define KSCAN_MAP_GPIO_ROW1           KEY_ROW_P02
#define KSCAN_MAP_GPIO_ROW2           KEY_ROW_P07
#define KSCAN_MAP_GPIO_ROW3           KEY_ROW_P34
#define KSCAN_MAP_GPIO_ROW4           KEY_ROW_P25

#define KSCAN_MAP_GPIO_COL0           KEY_COL_P01
#define KSCAN_MAP_GPIO_COL1           KEY_COL_P03
#define KSCAN_MAP_GPIO_COL2           KEY_COL_P11
#define KSCAN_MAP_GPIO_COL3           KEY_COL_P14
#define KSCAN_MAP_GPIO_COL4           KEY_COL_P24
#define KSCAN_MAP_GPIO_COL5           KEY_COL_P26

#endif

#endif








