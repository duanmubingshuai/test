/**************************************************************************************************
*******
**************************************************************************************************/

/******************************************************************************

 *****************************************************************************/

#include "types.h"

#ifdef __cplusplus
extern "C"
{
#endif


#define START_DEVICE_EVT                              0x0001

#define MOUSE_MOTION_STATUS_CHECK_LOOP_EVT            0X0002
#define MOUSE_MOVE_GET_XY_LOOP_EVT                    0X0004

#define MOUSE_N500MS_COUNT_EVT                    		0X0008

#define HID_TEST_EVT                                  0x0100
#define HID_MOUSE_SEND_EVT                            0x0200



typedef struct{
	uint8 button;
	uint8 x;
	uint8 y;
	int8 wheel;	
}HIDMOUSE_DATA;


typedef struct{
	uint8 id;
	uint8 status;
	uint8 cur_DPI;
	uint8 init_flag;	
}SENSOR;

enum{
	SENSOR_CPI_LOW_LEVEL = 1,
	SENSOR_CPI_MID_LEVEL,
	SENSOR_CPI_HIG_LEVEL,
	SENSOR_CPI_MAX_LEVEL,
};

enum {
    SENSOR_CPI_READ = 0,
    SENSOR_CPI_ADD,
    SENSOR_CPI_MIN,
	SENSOR_CPI_RST,
};

#define SENSOR_CPI_TOTAL_NUM	3		/// 1~3   cpi choise
#define SENSOR_CPI_DEFAULT		2		///  2   default middle

void   msensor_init(uint8 task_id);
uint16 msensor_process_event(uint8 task_id, uint16 events );
uint32 get_cur_sensor_data(void);
uint8 sensor_set_enable(void);
uint8 sensor_powerOnOff(uint8 powerFlag);
void sensor_set_change_cpi(void);

extern SENSOR HidMouseSensor;

#define UI_MOUSE_MOTION_STATUS_CHECK_LOOP_EVT						0x1000
