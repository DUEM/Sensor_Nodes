#include <SPI.h>
#include "DUEM_can.h"
#include "mcp_can.h"

INT32U global_id = CAN_GLOBAL_ID;
INT32U node_id = 0;
INT32U node_type = 0;
bool quiet = 0;

INT32U message_id = 0;
INT8U message_len = 0;
INT8U message_buf[8];

MCP_CAN CAN(10);


void duem_send_message(DUEMCANMessage msg) {
  
    message_id = node_id;
	message_len = 2;
    
    switch (msg.CommandId) {
        case ERROR_MESSAGE:
        message_len = 4;
        message_buf[message_len-3] = msg.ErrorId;
        message_buf[message_len-4] = msg.ErrorData;
        break;
        
        case DATA_TRANSMIT:
        message_len = 8;
        message_buf[message_len-3] = msg.DataFieldId;
        message_buf[message_len-4] = msg.Flags;
        for(int i=0; i<4; i++) { message_buf[i] = msg.DataFieldData.str[i]; }
        break;
        
        case DATA_REQUEST:
        message_len = 4;
        message_buf[message_len-3] = msg.DataFieldId;
        message_buf[message_len-4] = msg.Flags;
        break;
        
        case BROADCAST_REQUEST:
        message_len = 4;
        message_buf[message_len-3] = msg.DataFieldId;
        message_buf[message_len-4] = msg.Flags;
        break;
        
        case PARAMETER_SET:
        message_len = 8;
        message_buf[message_len-3] = msg.DataFieldId;
        message_buf[message_len-4] = msg.Flags;
        for(int  i=0; i<4; i++) { message_buf[i] = msg.DataFieldData.str[i]; }
        break;
        
        case PING:
        message_len = 2;
        break;
        
        case ACKNOWLEDGE:
        message_len = 2;
        break;
        
    }
	
	// five bits of Command ID and then first 3 bits of Target ID
    message_buf[message_len-1] = ((msg.CommandId << 3) | ((msg.TargetId & 0x700) >> 8));
    
    // last 8 bits of Target ID
    message_buf[message_len-2] = (msg.TargetId & 0xFF);
    
    CAN.sendMsgBuf(message_id, 0, message_len, message_buf);
}

DUEMCANMessage duem_rcv_message(INT32U id, INT8U len, byte* buf) {

    DUEMCANMessage msg;
	msg.CommandId = ((buf[len-1] >> 3) & 0x1F); //get first five bits
	msg.TargetId = (((buf[len-1] & 0x07) << 8) | buf[len-2]); //get next 11
	
	if(msg.CommandId == ERROR_MESSAGE){
		msg.ErrorId = (buf[len-3]);
		msg.ErrorData = (buf[len-4]);
		
		return msg;
	}
	
	else if(msg.CommandId == DATA_TRANSMIT || msg.CommandId == PARAMETER_SET){
		msg.DataFieldId = (buf[len-3]);
		msg.Flags = (buf[len-4]);
		
		for(int i=0; i<4; i++) { msg.DataFieldData.str[i] = message_buf[i]; }
		
		return msg;
	}
	
	else if(msg.CommandId == DATA_REQUEST || msg.CommandId == BROADCAST_REQUEST){
		msg.DataFieldId = (buf[len-3]);
		msg.Flags = (buf[len-4]);
		
		return msg;
	}
	
	else if(msg.CommandId == PING || msg.CommandId == ACKNOWLEDGE) {
		return msg;
	}
	
	else {
		//unknown message type
	}
	
}
