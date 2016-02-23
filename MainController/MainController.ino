#include <SPI.h>
#include "mcp_can.h"

////////////////////////////////////////////////
// Global CAN settings
////////////////////////////////////////////////

#define CAN_BAUD_RATE           CAN_500KBPS
#define CAN_GLOBAL_ID           0x400

#define CAN_GLOBAL_RST_ID       0x000

#define CAN_MASK                0x000  // We want all messages
#define CAN_FILTER              0x000  


////////////////////////////////////////////////
// Device Settings
////////////////////////////////////////////////

#define DEVICE_NODE_ID          MAIN_CONTROLLER_ID
#define DEVICE_RESET_ID         DEVICE_NODE_ID | 0x500

#define DEVICE_NODE_TYPE        MAIN_CONTROLLER

#define SHORT_TIMER_PERIOD      100     //millisecs
#define LONG_TIMER_PERIOD       1000   //millisecs

////////////////////////////////////////////////
// CAN Protocol Defines
////////////////////////////////////////////////

// Node IDs
#define MAIN_CONTROLLER_ID      0x402

// Node Type IDs
#define MAIN_CONTROLLER         1

// Message Headers
#define ERROR_MESSAGE           0b00000
#define DATA_TRANSMIT           0b10100
#define DATA_REQUEST            0b10101
#define BROADCAST_REQUEST       0b10110
#define PARAMETER_SET           0b10111
#define PING                    0b11110
#define ACKNOWLEDGE             0b11111

// Data Field IDs
#define FIELD_NODE_ID           1
#define ROAD_SPEED_S            61

////////////////////////////////////////////////
// Global Variables
////////////////////////////////////////////////

INT32U global_id = CAN_GLOBAL_ID;
INT32U node_id = DEVICE_NODE_ID;

INT32U node_type = DEVICE_NODE_TYPE;

bool quiet = 0;
float road_speed = 0; // road speed in m/s

float motor_set_speed = 30.0;
float motor_set_current = 0.0;
float bus_set_current = 0.5;

int i=0; //general counter
bool strobea=0; //var for heartbeat

long short_timer_last = 0;
long short_timer_period = SHORT_TIMER_PERIOD;
long long_timer_last = 0;
long long_timer_period = LONG_TIMER_PERIOD;


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

INT32U message_id = 0;
INT8U message_len = 0;
INT8U message_buf[8];

// Can Interface Object
MCP_CAN CAN(10); // Set CS to pin 10


////////////////////////////////////////////////

void send_message(DUEMCANMessage msg);

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
    CAN.init_Mask(1, 0, CAN_MASK);
    
    //Filters for RXB1
    CAN.init_Filt(2, 0, CAN_FILTER);
    CAN.init_Filt(3, 0, CAN_FILTER);
    CAN.init_Filt(4, 0, CAN_FILTER);
    CAN.init_Filt(5, 0, CAN_FILTER);
    
    CAN.enableBufferPins();
    
    // Send Wake-up message
    DUEMCANMessage msg_out;
    msg_out.CommandId = DATA_TRANSMIT;
    msg_out.TargetId = global_id;
    msg_out.DataFieldId = FIELD_NODE_ID;
    msg_out.Flags = 0;
    msg_out.DataFieldData.i = node_id;
    send_message(msg_out);
    
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
// Functions
////////////////////////////////////////////////


////////////////////////////////////////////////

void send_message(DUEMCANMessage msg) {
  
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

////////////////////////////////////////////////

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/