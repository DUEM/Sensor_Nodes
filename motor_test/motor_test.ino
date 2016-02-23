#include <DUEM_can.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <can_protocol.h>

#include <SPI.h>
#include "mcp_can.h"

#define DRIVER_CONTROL_BASE     0x500

#define SHORT_TIMER_PERIOD      100    //millisecs
#define LONG_TIMER_PERIOD       1000   //millisecs

unsigned char Flag_Recv = 0;

//Vars for incoming data
INT32U in_id = 0;
INT8U in_len = 0;
INT8U in_data[8];

//Vars for outgoing data from serial
INT32U out_id = 0;
INT8U out_len = 0;
INT8U out_data[8];

//Generic speed message
INT8U d_data[8] = {0,0,32,65,205,204,204,61};

//Vars for motor control
float motor_set_speed = 30.0;
float motor_set_current = 0.0;
float bus_set_current = 0.5;

long short_timer_last = 0;
long short_timer_period = SHORT_TIMER_PERIOD;
long long_timer_last = 0;
long long_timer_period = LONG_TIMER_PERIOD;

char str[20];

union eight_byte_data {
  float f[2]; INT8U c[8];
} s_data;

MCP_CAN CAN(10);                                            // Set CS to pin 10

void setup()
{
    Serial.begin(115200);

START_INIT:

    if(CAN_OK == CAN.begin(CAN_1000KBPS))                   // init can bus : baudrate = 500k
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
}


void loop()
{
    if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    {
        CAN.readMsgBufID(&in_id, &in_len, in_data);    // read data,  len: data length, buf: data buf
        
        Serial.print(">>> id: "); Serial.print(in_id); Serial.print(", data:");
        for(int i = 0; i < in_len; i++) { Serial.print(" "); Serial.print(in_data[i],HEX);}
        Serial.println();
    }
    
    while (Serial.available() > 0) {
      
        // look for the next valid integer in the incoming serial stream:
        out_id = Serial.parseInt();
        out_len = Serial.parseInt();
        
        if(out_len<0) out_len=0;
        if(out_len>8) out_len=8;
        
        for (int i=0; i < out_len; i++) { out_data[i] = Serial.parseInt(); }
    
        if (Serial.read() == '\n') {
            
            CAN.sendMsgBuf(out_id, 0, out_len, out_data);
            
            Serial.print("<<< id: "); Serial.print(out_id); Serial.print(", data:");
            for(int i = 0; i < out_len; i++) { Serial.print(" "); Serial.print(out_data[i],HEX);}
            Serial.println();
            
        }
    }
    
    ////////////////////////////////////////////////
    // SHORT LOOP
    ////////////////////////////////////////////////
    
    //Save value now to prevent value changing
    long milliseconds = millis();
    
    if ( (milliseconds >= short_timer_last + short_timer_period) || (milliseconds < short_timer_last) ) {
        //second condition just in case timer ticks over


        // Set Current
        motor_set_current = 0.4;// * analogRead(A0) / 1024;
        
        /////////////////
        // Send Velocity + Current Message

        motor_set_speed = motor_set_speed + 0.001;
        s_data.f[0] = motor_set_speed; //low float
        s_data.f[1] = motor_set_current; //high float
        
        CAN.sendMsgBuf(DRIVER_CONTROL_BASE+1, 0, 8, s_data.c);
 
        //print message
        Serial.print("<<< id: "); Serial.print(DRIVER_CONTROL_BASE+1); Serial.print(", data:");
        for(int i = 0; i < 8; i++) { Serial.print(" "); Serial.print(s_data.c[i],HEX);}
        Serial.println();
        
        /////////////////
        
        //reset last counter
        short_timer_last = millis();
    }
    
    ////////////////////////////////////////////////
    // LONG LOOP
    ////////////////////////////////////////////////
    
    milliseconds = millis();
    
    if ( (milliseconds >= long_timer_last + long_timer_period) || (milliseconds < long_timer_last) ) {
        
      
        /////////////////
        // Send Bus Current Message

        s_data.f[0] = 0.0f; //low float
        s_data.f[1] = bus_set_current; //high float
        
        CAN.sendMsgBuf(DRIVER_CONTROL_BASE+2, 0, 8, s_data.c);
 
        //print message
        Serial.print("<<< id: "); Serial.print(DRIVER_CONTROL_BASE+2); Serial.print(", data:");
        for(int i = 0; i < 8; i++) { Serial.print(" "); Serial.print(s_data.c[i],HEX);}
        Serial.println();
        
        /////////////////
                
        //reset last counter
        long_timer_last = millis();
    }
    //CAN.sendMsgBuf(0x501, 0, 8, d_data);
}
