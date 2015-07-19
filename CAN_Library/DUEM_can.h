#include <SPI.h>
#include "mcp_can.h"

////////////////////////////////////////////////

union FourByteData
{
    INT32U i;
    float f;
    char str[4];
};

union EightByteData {
    float f[2];
    INT8U c[8];
} eight_byte_data;

struct DUEMCANMessage {
    INT8U CommandId;
    INT32U TargetId;
    
    char Flags;
    
    INT8U ErrorId;
    INT8U ErrorData;
    
    INT8U DataFieldId;
    union FourByteData DataFieldData;
};

INT32U global_id = CAN_GLOBAL_ID;
INT32U node_id = DEVICE_NODE_ID;
INT32U node_type = DEVICE_NODE_TYPE;
bool quiet = 0;

INT32U message_id = 0;
INT8U message_len = 0;
INT8U message_buf[8];

// Can Interface Object
MCP_CAN CAN(10); // Set CS to pin 10


////////////////////////////////////////////////

void duem_send_message(DUEMCANMessage msg);
DUEMCANMessage duem_rcv_message();

////////////////////////////////////////////////
