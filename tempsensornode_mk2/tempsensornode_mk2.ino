#include <DallasTemperature.h>
#include <OneWire.h>
#include <Wire.h>

#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "mcp_can.h"
#include "can_consts.h"

////////////////////////////////////////////////
// Device Settings
////////////////////////////////////////////////

#define DEVICE_RESET_ID         TEMP_CUTOFF_RST_ID

#define SHORT_TIMER_PERIOD      100    //millisecs
#define LONG_TIMER_PERIOD       1000   //millisecs

////////////////////////////////////////////////
// Pins
////////////////////////////////////////////////

#define CAN_HW_ENABLE_PIN      10
#define ONE_WIRE_PIN           4
#define HEARTBEAT_LED_PIN      7

#define RELAY_ENABLE_PIN       2
#define RELAY_ENABLE_IND_PIN   8

////////////////////////////////////////////////
// Global Variables
////////////////////////////////////////////////

bool strobea=0; //var for heartbeat

float batt_avr_temp = 0.0f;
int batt_no_temps = 0;
float batt_max_temp = 0.0f;
int batt_num_max_temp = 0;

float batt_temp_threshold = 50.0f;
int tot_sensors_threshold = 1;

float bus_voltage = 0.0f;
float max_bus_voltage_threshold = 180.0f;
float min_bus_voltage_threshold = 140.0f;

#define BATT_NUM 0

DeviceAddress batt_add[3][12] = {
{ { 0x28, 0x17, 0xD5, 0xA0, 0x06, 0x00, 0x00, 0x34 },
{ 0x28, 0xFF, 0x58, 0x63, 0x68, 0x14, 0x02, 0x10 },
{ 0x28, 0xFF, 0x84, 0x96, 0x6C, 0x14, 0x04, 0xDC },
{ 0x28, 0xFF, 0x84, 0xF3, 0x67, 0x14, 0x03, 0x8F },
{ 0x28, 0xFF, 0x0C, 0xDD, 0x00, 0x15, 0x02, 0x5A },
{ 0x28, 0xFF, 0x0A, 0xA1, 0x68, 0x14, 0x03, 0x66 },
{ 0x28, 0xFF, 0x2A, 0xFE, 0x67, 0x14, 0x03, 0x83 },
{ 0x28, 0xFF, 0x1E, 0xB9, 0x67, 0x14, 0x03, 0x7E },
{ 0x28, 0xFF, 0x35, 0x64, 0x68, 0x14, 0x02, 0x6B },
{ 0x28, 0xFF, 0x6D, 0x9A, 0x6C, 0x14, 0x04, 0xE7 },
{ 0x28, 0xFF, 0x53, 0xFD, 0x67, 0x14, 0x03, 0x95 },
{ 0x28, 0xFF, 0xEB, 0xD7, 0x6C, 0x14, 0x04, 0xCD } },

{ { 0x28, 0x98, 0xC9, 0xA0, 0x06, 0x00, 0x00, 0xA7 },
{ 0x28, 0x2A, 0xBD, 0xA0, 0x06, 0x00, 0x00, 0xBC },
{ 0x28, 0x31, 0xD1, 0xEC, 0x06, 0x00, 0x00, 0xDD },
{ 0x28, 0xE9, 0xCE, 0xA0, 0x06, 0x00, 0x00, 0x59 },
{ 0x28, 0x39, 0xD0, 0xA0, 0x06, 0x00, 0x00, 0x43 },
{ 0x28, 0x79, 0xC0, 0xA0, 0x06, 0x00, 0x00, 0x4A },
{ 0x28, 0xE5, 0xB0, 0xA0, 0x06, 0x00, 0x00, 0xEB },
{ 0x28, 0x55, 0xB7, 0xA0, 0x06, 0x00, 0x00, 0xBD },
{ 0x28, 0x2D, 0xC6, 0xEC, 0x06, 0x00, 0x00, 0xD6 },
{ 0x28, 0x1D, 0xE8, 0xEC, 0x06, 0x00, 0x00, 0x61 },
{ 0x28, 0x0B, 0xC8, 0xEC, 0x06, 0x00, 0x00, 0x70 },
{ 0x28, 0x6F, 0xE3, 0xEC, 0x06, 0x00, 0x00, 0xE7 } },

{ { 0x28, 0x10, 0xCD, 0xEC, 0x06, 0x00, 0x00, 0x01 },
{ 0x28, 0xC4, 0xD8, 0xA0, 0x06, 0x00, 0x00, 0x45 },
{ 0x28, 0x54, 0xC1, 0xA0, 0x06, 0x00, 0x00, 0x7B },
{ 0x28, 0x1C, 0xBE, 0xA0, 0x06, 0x00, 0x00, 0xAD },
{ 0x28, 0x5C, 0xBA, 0xA0, 0x06, 0x00, 0x00, 0xC7 },
{ 0x28, 0x5A, 0xB1, 0xA0, 0x06, 0x00, 0x00, 0x05 },
{ 0x28, 0x3A, 0xD2, 0xA0, 0x06, 0x00, 0x00, 0x99 },
{ 0x28, 0xA1, 0xC7, 0xA0, 0x06, 0x00, 0x00, 0x7E },
{ 0x28, 0x63, 0xCD, 0xA0, 0x06, 0x00, 0x00, 0x32 },
{ 0x28, 0x17, 0xCE, 0xA0, 0x06, 0x00, 0x00, 0x38 },
{ 0x28, 0x1F, 0xBE, 0xA0, 0x06, 0x00, 0x00, 0xF4 },
{ 0x28, 0x3F, 0xBB, 0xA0, 0x06, 0x00, 0x00, 0x90 } } };

INT32U msg_id_avr[3] = { BATT_TEMP_1_AVR, BATT_TEMP_2_AVR, BATT_TEMP_3_AVR };
INT32U msg_id_avr_no[3] = { BATT_TEMP_1_NUM_SENSOR, BATT_TEMP_2_NUM_SENSOR, BATT_TEMP_3_NUM_SENSOR };
INT32U msg_id_max[3] = { BATT_TEMP_1_MAX, BATT_TEMP_2_MAX, BATT_TEMP_3_MAX };
INT32U msg_id_max_no[3] = { BATT_TEMP_1_MAX_SENSOR, BATT_TEMP_2_MAX_SENSOR, BATT_TEMP_3_MAX_SENSOR };

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

int j = BATT_NUM; //legacy

MCP_CAN CAN(CAN_HW_ENABLE_PIN);

OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature temp_sensors(&oneWire);

////////////////////////////////////////////////

void setup()
{
    Serial.begin(115200);
    
// Set up temperature sensors
  temp_sensors.begin();

////////////////////////////////////////////////
// Set up CAN interface and initialise various sensors
////////////////////////////////////////////////

int retry = 0;

START_INIT:

    if(CAN_OK == CAN.begin(CAN_BAUD_RATE)) // init can bus : baudrate = 500k, RX buf pins on
    {
        Serial.println("CAN BUS Shield init ok!");
    }
    else
    {
        Serial.println("CAN BUS Shield init fail");
        delay(100);
        if (retry < 10) { retry++; goto START_INIT; }
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
    
    //enable relay enable and indicator
    pinMode(RELAY_ENABLE_PIN, OUTPUT);
    pinMode(RELAY_ENABLE_IND_PIN, OUTPUT);    
    
    //and set it high to open darlington
    digitalWrite(RELAY_ENABLE_PIN, HIGH);
    digitalWrite(RELAY_ENABLE_IND_PIN, HIGH);

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
        
        Serial.print("Requesting temperatures...");
        temp_sensors.requestTemperatures(); // Send the command to get temperatures
        Serial.println("DONE");

        bool batt_status = 1;
        
        
        float max_temp = 0.0f; int num_max_temp = 0;
        float tot_temp = 0.0f; int num_temps = 0;

        Serial.print("Batt "); Serial.print(j+1); Serial.println(" temps:");
        for(int i=0; i<12; i++) {
          float tempC = temp_sensors.getTempC(batt_add[j][i]);
          //delay(50); //to reduce current load
          Serial.print(tempC); Serial.print("\t");
          if ((tempC <= 84.0) & (tempC > 0.1)){
            tot_temp = tot_temp + tempC; num_temps++;
            if (tempC > max_temp) { max_temp = tempC; num_max_temp = i; }
          }
        }
        
        batt_avr_temp = tot_temp / num_temps;
        batt_no_temps = num_temps;
        batt_max_temp = max_temp;
        batt_num_max_temp = num_max_temp;
        
        Serial.print("\nMax: "); Serial.print(max_temp);
        Serial.print("C @ #"); Serial.println(num_max_temp);
        Serial.print("Avr: "); Serial.print(batt_avr_temp);
        Serial.print(" of "); Serial.println(num_temps);
        
        if ((batt_max_temp > batt_temp_threshold) || (batt_no_temps < tot_sensors_threshold)) {
          //Battery too hot or sensors dead, disable input to darlington and open contactors
          digitalWrite(RELAY_ENABLE_PIN, LOW);
          digitalWrite(RELAY_ENABLE_IND_PIN, LOW);
          batt_status = 0;
          Serial.print("Temperature Error on Batt ");
          Serial.print(j+1); Serial.println(", Relays Opened!");

          //Send Error Message
          t_data.i[0] = 0; message_id = BATT_STATUS_ERROR;
          CAN.sendMsgBuf(message_id, 0, 4, t_data.c);
          
          delay(2000); // delay to turn off battery for at least 5s
        }
    
        // Send Messages
        t_data.f[0] = batt_avr_temp;
        message_id = msg_id_avr[j];
        CAN.sendMsgBuf(message_id, 0, 4, t_data.c);
        
        t_data.i[0] = batt_no_temps;
        message_id = msg_id_avr_no[j];
        CAN.sendMsgBuf(message_id, 0, 4, t_data.c);
        
        t_data.f[0] = batt_max_temp;
        message_id = msg_id_max[j];
        CAN.sendMsgBuf(message_id, 0, 4, t_data.c);
        
        t_data.i[0] = batt_num_max_temp;
        message_id = msg_id_max_no[j];
        CAN.sendMsgBuf(message_id, 0, 4, t_data.c);

        if(batt_status == 1) {
          //All batts Okay, we can close relays again.
          digitalWrite(RELAY_ENABLE_PIN, HIGH);
          digitalWrite(RELAY_ENABLE_IND_PIN, HIGH);
        }
        
        //reset last counter
        long_timer_last = millis();
    }
    
    
    ////////////////////////////////////////////////
    // CAN message handler
    ////////////////////////////////////////////////
    

    while(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    {
        CAN.readMsgBufID(&message_id, &message_len, t_data.c);

        if (message_id == BATT_STATUS_ERROR) {
          //Error from one of the other btteries so open relays
          digitalWrite(RELAY_ENABLE_PIN, LOW);
          digitalWrite(RELAY_ENABLE_IND_PIN, LOW);
          
          delay(3000);
          
          //flush messagees
          while(CAN_MSGAVAIL == CAN.checkReceive())  {
            CAN.readMsgBufID(&message_id, &message_len, message_buf);
          }
        }

        #if BATT_NUM == 0
        if (message_id == MOTOR_CONTROLLER_BASE+2) {
          bus_voltage = t_data.f[0]; //low float
          if (bus_voltage > max_bus_voltage_threshold) {
            t_data.i[0] = 1; message_id = BATT_STATUS_ERROR;
            CAN.sendMsgBuf(message_id, 0, 4, t_data.c);

            digitalWrite(RELAY_ENABLE_PIN, LOW);
            digitalWrite(RELAY_ENABLE_IND_PIN, LOW);
            
            delay(3000);
          }

          if (bus_voltage < min_bus_voltage_threshold) {
            t_data.i[0] = 2; message_id = BATT_STATUS_ERROR;
            CAN.sendMsgBuf(message_id, 0, 4, t_data.c);

            digitalWrite(RELAY_ENABLE_PIN, LOW);
            digitalWrite(RELAY_ENABLE_IND_PIN, LOW);
            
            delay(3000);
          }
        }
        #endif
        
    }
    
}

////////////////////////////////////////////////

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
