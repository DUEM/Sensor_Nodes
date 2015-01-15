// DUEM - Solar Car 2014-2015 
// sensor node programme
// when in interrupt mode, the data coming can't be too fast, must >20ms, or else you can use check mode
// 

#include <SPI.h>
#include "mcp_can.h"
#define MYID 100
#define GLOBALID 1000
#define ERROR_MESSAGE 0
#define DATA_TRANSMIT 20
#define DATA_REQUEST 21
#define BROADCAST_REQUEST 22
#define PARAMETER_SET 23
#define PING 30
#define ACKNOWLEDGE 31
/* 
could also define if the last 3 bits are 111 then its some sort of recalibration message.
*/

struct DUEMCANMessage {
	
	int CommandID;
	int TargetId;
	
	char Flags;
	
	short DataFieldID;
	long DataFieldData;
	
}

/* Variable Definitions */
int TargetID,
	CommandID,
	DataFieldID,
	Reserved,
	Frequency,
	RecievedMessageID,
	SendMessageID,
	RemoteFrame;


MCP_CAN CAN(10);                                            // Set CS to pin 10

unsigned char Flag_Recv = 0;
unsigned char len = 0;
unsigned char buf[8];
unsigned char message[8] 				// message to be sent
INT32U send_message_id;						// send id to message
INT32U recieved_message_id;					// message id recieved
INT8U remote_frame;					// whether the message is a remote frame or not
INT32U sender_id;
INT8U message_length;
INT8U message_data[8];
int test = 0;

char str[20];

void send_data() //get it to return something to check if it succeeds. give it an input for the data and node to send

void setup()
{
    Serial.begin(115200);

START_INIT:

    if(CAN_OK == CAN.begin(CAN_500KBPS))                   // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init ok!");
    }
    else
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println("Init CAN BUS Shield again");
        delay(100);
        goto START_INIT;
    }
    
    /*
     * set mask, set both the mask to 0x3ff
     */
    CAN.init_Mask(0, 0, 0x3FF);                         // there are 2 mask in mcp2515, you need to set both of them
    CAN.init_Mask(1, 0, 0x3FF); 
    /*
     * set filter, we can receive id from 0x04 ~ 0x09
     */
    CAN.init_Filt(0, 0, 0x04);                          // there are 6 filter in mcp2515
    CAN.init_Filt(1, 0, 0x05);                          // there are 6 filter in mcp2515
    
    CAN.init_Filt(2, 0, 0x06);                          // there are 6 filter in mcp2515
    CAN.init_Filt(3, 0, 0x07);                          // there are 6 filter in mcp2515
    CAN.init_Filt(4, 0, 0x08);                          // there are 6 filter in mcp2515
    CAN.init_Filt(5, 0, 0x09);                          // there are 6 filter in mcp2515

}

void send_data()
{
    // send data:  id = 0x00, standrad flame, data len = 8, stmp: data buf
    CAN.sendMsgBuf(send_message_id, 0, 8, message);
    delay(100);                       // send data per 100ms
}

int getspeed(){
	// temp for testing replace with actual stuff
	test = test + 1
	return test
}

void loop()
{
	
	 if(CAN_MSGAVAIL == CAN.checkReceive())             // check if data coming
   		{
    
        	FlagRecv = 0;     // clear flag
        	CAN.readMsgBufID(*sender_id,*message_length,*message_data);			// read data,  len: data length, buf: data 
		CommandID   = ((message_data[0] >> 3) & 0x1F); 					//getting first five bits
   	        TargetID    = (((message_data[0] & 0x07) << 8) || message_data[1]);
   		 if (TargetID == MyID || TargetID == GlobalID)  
   		 {
   		        if(CommandID == ERROR_MESSAGE){
   		  		ErrorID   = (message_data[2]);
   		  		ErrorData = (message_data[3]);
   		        }
   		        // CHECK EVERYTHING BELOW THIS POINT IS OK
   		        // Can definitely be shortened
   		        else if(CommandID == DATA_TRANSMIT){
   		  		DataFieldID = (message_data[2]);
   		  		Flags = (message_data[3]);
   		  		DataFieldData = (message_data[4]); // Definitely wrong
   		  		// save data somewhere if needed
   		  		// write function to call
   		        }
   		        else if(CommandID == DATA_REQUEST){
   		  		DataFieldID = (message_data[2]);
   		  		Flags = (message_data[3]);
   		  		if (DataFieldID == 61){ // request for speed
   		  			DUEMCANMessage msg;
   		  			msg.CommandID = DATA_TRANSMIT;
   		  			msg.TargetId = GlobalID; // change to return to sender only
   		  			msg.DataFieldID = 61;
   		  			msg.Flags = 0;
   		  			msg.DataFieldData = (long) getspeed();
   		  			send(msg) // write function to send can message
   		  		}
   		        }
   		        else if(CommandID == BROADCAST_REQUEST){
   		  		DataFieldID = (message_data[2]);
   		  		Flags = (message_data[3]);
   		        }
   		        else if(CommandID == PARAMETER_SET){
   		  		DataFieldID = (message_data[2]);
   		  		Flags = (message_data[3]);
   		  		DataFieldData = (message_data[4]); // Definitely wrong
   		  		// write function to update parameter if needed
   		        }
   		        else if(CommandID == PING){
   		  		
   		        }
   		        
   			DataFieldID  = (RecievedMessageID & 0x03FC0000)	>> 16;
   			
   			Reserved     = (RecievedMessageID & 0x3C000000)	>> 24;
   			Frequebuwyuevwcucweuycgerrrrrrrrrwncy    = (RecievedMessageID & 0xF8000000)	>> 28;
		}
		
		
	if (TargetID == MyID || TargetID == 0) //global case = 0, node specific = MyID
			{
				//do something
				sendData();
			}

    
	
	}
		
       /* Serial.println("\r\n------------------------------------------------------------------");
        Serial.print("Get Data From id: ");
        Serial.println(CAN.getCanId());
      
	   for(int i = 0; i<len; i++)    // print the data for debugging purposes
        {
            Serial.print("0x");
            Serial.print(buf[i], HEX);
            Serial.print("\t");
        }
        Serial.println();	
		*/
    }
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
