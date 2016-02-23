#include <SPI.h>
#include "mcp_can.h"
#include "LedControl.h"
#include "can_consts.h"

////////////////////////////////////////////////
// Device Settings
////////////////////////////////////////////////

#define DEVICE_RESET_ID         MAIN_CONTROLLER_RST_ID

#define SHORT_TIMER_PERIOD      100    //millisecs
#define LONG_TIMER_PERIOD       1000   //millisecs

////////////////////////////////////////////////
// Pins
////////////////////////////////////////////////

#define CAN_HW_ENABLE_PIN      10
#define HEARTBEAT_LED_PIN      7

#define DISP_DATA_PIN          4
#define DISP_CLK_PIN           3
#define DISP_CS_PIN            2

////////////////////////////////////////////////
// Global Variables
////////////////////////////////////////////////

bool strobea=0; //var for heartbeat

float car_speed = 0.0f;
int batt_error = 0;
int batt_error_flags = 0;
float telemetry_value = 0.0f;

long short_timer_last = 0;
long short_timer_period = SHORT_TIMER_PERIOD;
long long_timer_last = 0;
long long_timer_period = LONG_TIMER_PERIOD;

union {
  INT8U c[8];
  float f[2];
  INT32U i[2];
} eight_byte_data;

INT32U message_id;
INT8U message_len;
INT8U message_buf[8];

MCP_CAN CAN(CAN_HW_ENABLE_PIN);
LedControl ddisplay = LedControl(DISP_DATA_PIN, DISP_CLK_PIN, DISP_CS_PIN);

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

    //set up 7segs
    ddisplay.shutdown(0,false);
    ddisplay.setIntensity(0,2);
    ddisplay.clearDisplay(0);

    //enable LED for heartbeat
    pinMode(HEARTBEAT_LED_PIN, OUTPUT); 

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
        digitalWrite(HEARTBEAT_LED_PIN, strobea);
        

        //Update Display

        // Update Speed
        
        //ddisplay.setChar(0,7,'S',false);
        //ddisplay.setChar(0,6,'P',false);

         //set up 7segs
        ddisplay.shutdown(0,true);
        ddisplay.shutdown(0,false);
        ddisplay.setIntensity(0,2);
        ddisplay.clearDisplay(0);
    
        ddisplay.clearDisplay(0);
        int huns = int(car_speed/100.0)%10;
        int tens = int(car_speed/10.0)%10;
        int units = int(car_speed)%10;
        int tenths = int(car_speed*10)%10;
        if (huns) ddisplay.setDigit(0,7,huns,false);
        if (tens) ddisplay.setDigit(0,6,tens,false);
        ddisplay.setDigit(0,5,units,true);
        ddisplay.setDigit(0,4,tenths,false);

        huns = int(telemetry_value/100.0)%10;
        tens = int(telemetry_value/10.0)%10;
        units = int(telemetry_value)%10;
        tenths = int(telemetry_value*10)%10;
        if (huns) ddisplay.setDigit(0,3,huns,false);
        if (tens) ddisplay.setDigit(0,2,tens,false);
        ddisplay.setDigit(0,1,units,true);
        ddisplay.setDigit(0,0,tenths,false);


        if(batt_error){
          ddisplay.clearDisplay(0);
          ddisplay.setRow(0,7,0x7F);
          ddisplay.setRow(0,6,0x77);
          ddisplay.setRow(0,5,0x0F);
          ddisplay.setRow(0,4,0x0F);
  
          ddisplay.setRow(0,3,0x4F);
          ddisplay.setRow(0,2,0x05);
          ddisplay.setRow(0,1,0x05);
          ddisplay.setDigit(0,0,batt_error_flags,false);
          delay(5000);
          batt_error = 0;
        }
        
        //reset last counter
        long_timer_last = millis();
    }
    
    
    ////////////////////////////////////////////////
    // CAN message handler
    ////////////////////////////////////////////////
    
    if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    {
        float floaty[2] = { 0.0f, 0.0f };
        CAN.readMsgBufID(&message_id, &message_len, eight_byte_data.c);

         if (message_id == MOTOR_CONTROLLER_BASE+3) {
            //Set values
            car_speed = eight_byte_data.f[1]*3.6;
         }

         if (message_id == BATT_STATUS_ERROR) {
            batt_error = 1;
            batt_error_flags = eight_byte_data.i[0];
         }

         if ((message_id == BATT_TEMP_1_AVR) || (message_id == BATT_TEMP_2_AVR) || (message_id == BATT_TEMP_3_AVR)) {
            //Set values
            telemetry_value = eight_byte_data.f[0];
         }
        
        /*
        // Print message to serial for debugging
        Serial.print(message_id); Serial.print(":\t");
        for(int i = 0; i < message_len; i++) {
            Serial.print(message_buf[i]); Serial.print(" ");
        }
        Serial.println();*/
        
    }
    
}

////////////////////////////////////////////////

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
