// DUEM - Solar Car 2014-2015 
// sensor node programme
// when in interrupt mode, the data coming can't be too fast, must >20ms, or else you can use check mode
// 

#include <SPI.h>
#include "mcp_can.h"
#define MESSAGE_ID_ERROR_BITCH_PLEASE 0
#define BITCH_PLEASE 20
#define WOOF_BACK 21
#define BITCHES_BE_CRAZY 22
#define VIVA_LA_REVELOUTION 23
#define GROWLING 30
#define SANDWICH_TIME 31
/* 
could also define if the last 3 bits are 111 then its some sort of recalibration message.
*/

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


void loop()
{
	
	 if(CAN_MSGAVAIL == CAN.checkReceive())             // check if data coming
   		{
    
        	FlagRecv = 0;     // clear flag
        	CAN.readMsgBufID(*sender_id,*message_length,*message_data);	// read data,  len: data length, buf: data 
			CommandID     = ((message_data[0] >> 3) & 0x1F) ; //getting first five bits
   		        TargetID    = (((message_data[0] & 0x07) << 8) || message_data[1]);
   		        if(CommandID == MESSAGE_ID_ERROR_BITCH_PLEASE){
   		  			
   		        }
   		        else if(CommandID == BITCH_PLEASE){
   		  			
   		        }
   		        else if(CommandID == WOOF_BACK){
   		  			
   		        }
   		        else if(CommandID == BITCHES_BE_CRAZY){
   		  		
   		        }
   		        else if(CommandID == VIVA_LA_REVELOUTION){
   		  			
   		        }
   		        else if(CommandID == GROWLING){
   		  			
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
