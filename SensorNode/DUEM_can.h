//#ifndef _DUEM_can_H_
//#define _DUEM_can_H_

#include <SPI.h>
#include "mcp_can.h"
#include "can_protocol.h"


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
} extern eight_byte_data;

struct DUEMCANMessage {
    INT8U CommandId;
    INT32U TargetId;
    
    char Flags;
    
    INT8U ErrorId;
    INT8U ErrorData;
    
    INT8U DataFieldId;
    union FourByteData DataFieldData;
};

extern INT32U global_id;
extern INT32U node_id;
extern INT32U node_type;
extern bool quiet;

extern INT32U message_id;
extern INT8U message_len;
extern INT8U message_buf[8];

extern MCP_CAN CAN;

////////////////////////////////////////////////

void duem_send_message(DUEMCANMessage msg);
DUEMCANMessage duem_rcv_message(INT32U id, INT8U len, byte* buf);

////////////////////////////////////////////////

//#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
