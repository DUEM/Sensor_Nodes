////////////////////////////////////////////////
// CAN Protocol Defines
////////////////////////////////////////////////

#define CAN_BAUD_RATE           CAN_1000KBPS


///////////////////////////////////////////////
// Node IDs

#define CAN_GLOBAL_RST_ID       0x000
#define MAIN_CONTROLLER_RST_ID  0x001
#define TEMP_CUTOFF_RST_ID      0x002

#define CAN_DEFAULT_MASK        0x000
#define CAN_DEFAULT_FILTER	0xFFF

///////////////////////////////////////////////
// Data IDs

#define BATT_TEMP_1_AVR         0x640 // 4 Byte Float - Deg C - Average temp of all sensors
#define BATT_TEMP_1_NUM_SENSOR  0x641 // 4 Byte Int - Number of sensors working
#define BATT_TEMP_1_MAX         0x642 // 4 Byte Float - Deg C - Max reported temp of all sensors
#define BATT_TEMP_1_MAX_SENSOR  0x643 // 4 Byte Int - Sensor reporting max reading

#define BATT_TEMP_2_AVR         0x644 // 4 Byte Float - Deg C - Average temp of all sensors
#define BATT_TEMP_2_NUM_SENSOR  0x645 // 4 Byte Int - Number of sensors working
#define BATT_TEMP_2_MAX         0x646 // 4 Byte Float - Deg C - Max reported temp of all sensors
#define BATT_TEMP_2_MAX_SENSOR  0x647 // 4 Byte Int - Sensor reporting max reading

#define BATT_TEMP_3_AVR         0x648 // 4 Byte Float - Deg C - Average temp of all sensors
#define BATT_TEMP_3_NUM_SENSOR  0x649 // 4 Byte Int - Number of sensors working
#define BATT_TEMP_3_MAX         0x64A // 4 Byte Float - Deg C - Max reported temp of all sensors
#define BATT_TEMP_3_MAX_SENSOR  0x64B // 4 Byte Int - Sensor reporting max reading

#define BATT_CURRENT_1          0x650 // 4 Byte Float - Amps
#define BATT_CURRENT_2          0x651 // 4 Byte Float - Amps
#define BATT_CURRENT_3          0x652 // 4 Byte Float - Amps
#define BATT_CHARGE_COUNT_1     0x653 // 4 Byte Float - As - Charge since last message
#define BATT_CHARGE_COUNT_2     0x654 // 4 Byte Float - As - Charge since last message
#define BATT_CHARGE_COUNT_3     0x655 // 4 Byte Float - As - Charge since last message

#define BATT_VOLTAGE_1          0x656 // 4 Byte Float - V
#define BATT_VOLTAGE_2          0x657 // 4 Byte Float - V
#define BATT_VOLTAGE_3          0x658 // 4 Byte Float - V

#define DRIVER_CONTROLS_BASE    0x500
#define MOTOR_CONTROLLER_BASE   0x600
