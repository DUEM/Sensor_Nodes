#include <SPI.h>
#include "mcp_can.h"

#include "deum_can.h"

void duem_send_message(DUEMCANMessage msg) {
  
    message_id = node_id;
    
    // five bits of Command ID and then first 3 bits of Target ID
    message_buf[0] = ((msg.CommandId << 3) | ((msg.TargetId & 0x700) >> 8));
    
    // last 8 bits of Target ID
    message_buf[1] = (msg.TargetId & 0xFF);
    
    switch (msg.CommandId) {
        case ERROR_MESSAGE:
        message_len = 4;
        message_buf[2] = msg.ErrorId;
        message_buf[3] = msg.ErrorData;
        break;
        
        case DATA_TRANSMIT:
        message_len = 8;
        message_buf[2] = msg.DataFieldId;
        message_buf[3] = msg.Flags;
        for(i=0; i<4; i++) { message_buf[i+4] = msg.DataFieldData.str[3-i]; }
        break;
        
        case DATA_REQUEST:
        message_len = 4;
        message_buf[2] = msg.DataFieldId;
        message_buf[3] = msg.Flags;
        break;
        
        case BROADCAST_REQUEST:
        message_len = 4;
        message_buf[2] = msg.DataFieldId;
        message_buf[3] = msg.Flags;
        break;
        
        case PARAMETER_SET:
        message_len = 8;
        message_buf[2] = msg.DataFieldId;
        message_buf[3] = msg.Flags;
        for(i=0; i<4; i++) { message_buf[i+4] = msg.DataFieldData.str[3-i]; }
        break;
        
        case PING:
        message_len = 2;
        break;
        
        case ACKNOWLEDGE:
        message_len = 2;
        break;
        
    }
    
    CAN.sendMsgBuf(message_id, 0, message_len, message_buf);
}
