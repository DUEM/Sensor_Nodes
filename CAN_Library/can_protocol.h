////////////////////////////////////////////////
// CAN Protocol Defines
////////////////////////////////////////////////

#define CAN_BAUD_RATE           CAN_1000KBPS

#define CAN_GLOBAL_ID           0x400
#define CAN_GLOBAL_RST_ID       0x000

#define CAN_DEFAULT_MASK        0xF00
#define CAN_DEFAULT_FILTER      0x400

////////////////////////////////////////////////

// Node IDs
#define MAIN_CONTROLLER_ID      0x402
#define WHEEL_SPEED_SENSOR_ID   0x420

#define DRIVER_CONTROL_BASE     0x500
#define MOTOR_CONTROL_BASE      0x600

////////////////////////////////////////////////

// Node Types
#define MAIN_CONTROLLER         1
#define WHEEL_SPEED_SENSOR      20

////////////////////////////////////////////////

// Message Headers
#define ERROR_MESSAGE           0b00000
#define DATA_TRANSMIT           0b10100
#define DATA_REQUEST            0b10101
#define BROADCAST_REQUEST       0b10110
#define PARAMETER_SET           0b10111
#define PING                    0b11110
#define ACKNOWLEDGE             0b11111

////////////////////////////////////////////////

// Data Field IDs
#define FIELD_NODE_ID           1
#define ROAD_SPEED_S            61

////////////////////////////////////////////////
