#include <SPI.h>
#include "mcp_can.h"
#include "can_consts.h"

////////////////////////////////////////////////
// Device Settings
////////////////////////////////////////////////

#define DEVICE_RESET_ID         CAN_GLOBAL_RST_ID

#define SHORT_TIMER_PERIOD      100    //millisecs
#define LONG_TIMER_PERIOD       1000   //millisecs

////////////////////////////////////////////////
// Pins
////////////////////////////////////////////////

#define CAN_HW_ENABLE_PIN      10
#define HEARTBEAT_LED_PIN      7

#define CURR_SENSE_1_PIN       A0
#define CURR_SENSE_2_PIN       A1
#define CURR_SENSE_3_PIN       A2

/*
// These constants won't change.  They're used to give names
// to the pins used:
const int analogPin1 = A0;  // Analog input pin that the potentiometer is attached to
const int analogPin2 = A1;
const int analogPin3 = A2;*/


////////////////////////////////////////////////
// Global Variables
////////////////////////////////////////////////

bool strobea=0; //var for heartbeat

const int currentRate = 1000; //milliseconds between current data
const int chargeRate = 1000*10; //milliseconds between charge data

// ADC config
const int readRez = 10;
const float maxADCVal = powf(2,readRez)-1 ;

// Current sensor information
float noCurrentVoltage1 = 2.5; //expected voltage when current is 0
float noCurrentVoltage2 = 2.5;
float noCurrentVoltage3 = 2.5;
float maxVolt = 5.0;
float ampRange1 = 300.0;  //maximum positive ampage
float ampRange2 = 300.0;
float ampRange3 = 300.0;


float sensor1Value;
float sensor2Value;
float sensor3Value;

float sensor1avg;
float sensor2avg;
float sensor3avg;

float sensor1curr;
float sensor2curr;
float sensor3curr;


INT32U message_id;
INT8U message_len;
INT8U message_buf[8];

long short_timer_last = 0;
long short_timer_period = SHORT_TIMER_PERIOD;
long long_timer_last = 0;
long long_timer_period = LONG_TIMER_PERIOD;

union {
  INT8U c[8];
  float f[2];
  INT32U i[2];
} t_data;

MCP_CAN CAN(CAN_HW_ENABLE_PIN);

////////////////////////////////////////////////

void setup() {
  // initialize serial communications at 115200 bps:
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
    
    //enable LED for heartbeat
    pinMode(HEARTBEAT_LED_PIN, OUTPUT);
    
}

void loop() {
    unsigned long chargeTime = millis();
    
    float dCharge1 = 0;
    float dCharge2 = 0;
    float dCharge3 = 0;
    while (millis()-chargeTime < chargeRate || millis()<chargeTime) //handle rollover
    { 
        unsigned long currTime = millis();
        int samples=0;
        float sensor1Value = 0;        
        float sensor2Value = 0;
        float sensor3Value = 0;

        while (millis()-currTime<currentRate || millis()<currTime ) //handle rollover
        {
            sensor1Value += analogRead(CURR_SENSE_1_PIN);
            sensor2Value += analogRead(CURR_SENSE_2_PIN);
            sensor3Value += analogRead(CURR_SENSE_3_PIN);
            samples++;
            delay(10); //sleep 2 milliseconds to allow ADC to recover
        }
        sensor1avg = sensor1Value/float(samples);
        sensor2avg = sensor2Value/float(samples);
        sensor3avg = sensor3Value/float(samples);
        
        sensor1curr = ampRange1*((sensor1avg/maxADCVal)-noCurrentVoltage1/maxVolt);
        sensor2curr = ampRange2*((sensor2avg/maxADCVal)-noCurrentVoltage2/maxVolt);
        sensor3curr = ampRange3*((sensor3avg/maxADCVal)-noCurrentVoltage3/maxVolt);
        Serial.print(sensor1curr);
        Serial.print("\t");
        Serial.print(sensor2curr);
        Serial.print("\t");
        Serial.println(sensor3curr); //send CAN message about current here

        t_data.f[0] = sensor1curr;
        message_id = BATT_CURRENT_1;
        CAN.sendMsgBuf(message_id, 0, 4, t_data.c);
        
        t_data.f[0] = sensor2curr;
        message_id = BATT_CURRENT_2;
        CAN.sendMsgBuf(message_id, 0, 4, t_data.c);
        
        t_data.f[0] = sensor3curr;
        message_id = BATT_CURRENT_3;
        CAN.sendMsgBuf(message_id, 0, 4, t_data.c);
          
        //Update Heartbeat
        strobea = 1-strobea;
        digitalWrite(HEARTBEAT_LED_PIN, strobea);
        
        dCharge1 += sensor1curr * currentRate/1000.0;
        dCharge2 += sensor2curr * currentRate/1000.0;
        dCharge3 += sensor3curr * currentRate/1000.0;
        
    }
    Serial.print(dCharge1);
    Serial.print("\t");
    Serial.print(dCharge2);
    Serial.print("\t");
    Serial.println(dCharge3); //send CAN message about charge here

    t_data.f[0] = dCharge1;
    message_id = BATT_CHARGE_COUNT_1;
    CAN.sendMsgBuf(message_id, 0, 4, t_data.c);
    
    t_data.f[0] = dCharge2;
    message_id = BATT_CHARGE_COUNT_2;
    CAN.sendMsgBuf(message_id, 0, 4, t_data.c);
    
    t_data.f[0] = dCharge3;
    message_id = BATT_CHARGE_COUNT_3;
    CAN.sendMsgBuf(message_id, 0, 4, t_data.c);
}
