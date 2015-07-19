#include <SPI.h>
#include "mcp_can.h"
#include "can_protocol.h"
#include "duem_can.h"

////////////////////////////////////////////////
// Device Settings
////////////////////////////////////////////////

#define DEVICE_NODE_ID          MAIN_CONTROLLER_ID
#define DEVICE_RESET_ID         DEVICE_NODE_ID | 0x500

#define DEVICE_NODE_TYPE        MAIN_CONTROLLER

#define SHORT_TIMER_PERIOD      100     //millisecs
#define LONG_TIMER_PERIOD       1000   //millisecs

////////////////////////////////////////////////
// Global Variables
////////////////////////////////////////////////

int i=0; //general counter
bool strobea=0; //var for heartbeat

long short_timer_last = 0;
long short_timer_period = SHORT_TIMER_PERIOD;
long long_timer_last = 0;
long long_timer_period = LONG_TIMER_PERIOD;

////////////////////////////////////////////////

void setup()
{
    Serial.begin(115200);

////////////////////////////////////////////////
// Set up CAN interface and initialise various sensors
////////////////////////////////////////////////

START_INIT:

    if(CAN_OK == CAN.begin(CAN_BAUD_RATE)) // init can bus : baudrate = 500k, RX buf pins on
    {
        Serial.println("CAN BUS Shield init ok!");
    }
    else
    {
        Serial.println("CAN BUS Shield init fail");
        delay(100);
        goto START_INIT;
    }
    
    
    //Mask for RXB0
    CAN.init_Mask(0, 0, 0xFFF);
    
    //Filters for RXB0
    CAN.init_Filt(0, 0, CAN_GLOBAL_RST_ID );
    CAN.init_Filt(1, 0, DEVICE_RESET_ID );

    //Mask for RXB1
    CAN.init_Mask(1, 0, CAN_DEFAULT_MASK);
    
    //Filters for RXB1
    CAN.init_Filt(2, 0, CAN_DEFAULT_FILTER);
    CAN.init_Filt(3, 0, CAN_DEFAULT_FILTER);
    CAN.init_Filt(4, 0, CAN_DEFAULT_FILTER);
    CAN.init_Filt(5, 0, CAN_DEFAULT_FILTER);
    
    CAN.enableBufferPins();
    
    // Send Wake-up message
    DUEMCANMessage msg_out;
    msg_out.CommandId = DATA_TRANSMIT;
    msg_out.TargetId = global_id;
    msg_out.DataFieldId = FIELD_NODE_ID;
    msg_out.Flags = 0;
    msg_out.DataFieldData.i = node_id;
    duem_send_message(msg_out);
    
    //enable LED for heartbeat
    pinMode(7, OUTPUT);

}

////////////////////////////////////////////////

void loop()
{
    
    ////////////////////////////////////////////////
    // SHORT LOOP
    ////////////////////////////////////////////////
    
    //Save value now to prevent value changing
    long milliseconds = millis();
    if ( (milliseconds >= short_timer_last + short_timer_period) || (milliseconds < short_timer_last) ) {
        //second condition just in case timer ticks over

        //reset last counter
        short_timer_last = millis();
    }
    
    ////////////////////////////////////////////////
    // LONG LOOP
    ////////////////////////////////////////////////
    
    milliseconds = millis();
    if ( (milliseconds >= long_timer_last + long_timer_period) || (milliseconds < long_timer_last) ) {
        
        //Update Heartbeat
        strobea = 1-strobea;
        digitalWrite(7, strobea);
        
        //reset last counter
        long_timer_last = millis();
    }
    
    
    ////////////////////////////////////////////////
    // CAN message handler
    ////////////////////////////////////////////////
    
    if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    {
        CAN.readMsgBufID(&message_id, &message_len, message_buf);
        
        DUEMCANMessage msg;
        
        msg.CommandId = ((message_buf[0] >> 3) & 0x1F); //get first five bits
        msg.TargetId = (((message_buf[0] & 0x07) << 8) | message_buf[1]); //get next 11
        
        if (msg.TargetId == node_id || msg.TargetId == global_id) {
          
            if(msg.CommandId == ERROR_MESSAGE){
                msg.ErrorId = (message_buf[2]);
                msg.ErrorData = (message_buf[3]);
                
                // Do something maybe
            }
            
            else if(msg.CommandId == DATA_TRANSMIT){
                msg.DataFieldId = (message_buf[2]);
                msg.Flags = (message_buf[3]);
                
                for(i=0; i<4; i++) { msg.DataFieldData.str[3-i] = message_buf[i+4]; }
                
                // save data somewhere if needed
            }
            
            else if(msg.CommandId == DATA_REQUEST){
                msg.DataFieldId = (message_buf[2]);
                msg.Flags = (message_buf[3]);
                
                // respond to request
                if (msg.DataFieldId == FIELD_NODE_ID){ // request for speed
                    DUEMCANMessage msg_out;
                    msg_out.CommandId = DATA_TRANSMIT;
                    msg_out.TargetId = global_id; // change to return to sender only
                    msg_out.DataFieldId = FIELD_NODE_ID;
                    msg_out.Flags = 0;
                    msg_out.DataFieldData.i = node_id;
                    send_message(msg_out);
                }
                
            }
            
            else if(msg.CommandId == BROADCAST_REQUEST){
                msg.DataFieldId = (message_buf[2]);
                msg.Flags = (message_buf[3]);
                
                // respond to request
            }
            
            else if(msg.CommandId == PARAMETER_SET){
                msg.DataFieldId = (message_buf[2]);
                msg.Flags = (message_buf[3]);
                
                for(i=0; i<4; i++) { msg.DataFieldData.str[3-i] = message_buf[i+4]; }
                
                // write function to update parameter if needed
            }
            
            else if(msg.CommandId == PING){
                // send ACK message back
                DUEMCANMessage msg_out;
                msg_out.CommandId = ACKNOWLEDGE;
                msg_out.TargetId = global_id; // change to return to sender only
                send_message(msg_out);
            }
            
            else if(msg.CommandId == ACKNOWLEDGE){
                // maybe do something
            }
            
            else {
                //unknown message type
            }
            
        }
        
        // Print message to serial for debugging
        Serial.print(message_id); Serial.print(":\t");
        for(int i = 0; i < message_len; i++) {
            Serial.print(message_buf[i]); Serial.print(" ");
        }
        Serial.println();
        
    }
    
    
}

////////////////////////////////////////////////

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
