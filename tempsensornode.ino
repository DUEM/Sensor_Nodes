#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "mcp_can.h"
#include "can_protocol.h"
#include "DUEM_can.h"

////////////////////////////////////////////////
// Device Settings
////////////////////////////////////////////////

#define DEVICE_NODE_ID          BATT_TEMP_MONITOR_ID
#define DEVICE_RESET_ID         DEVICE_NODE_ID + SENSOR_NODE_ID_RANGE

#define DEVICE_NODE_TYPE        BATTERY_MONITOR

#define SHORT_TIMER_PERIOD      100    //millisecs
#define LONG_TIMER_PERIOD       1000   //millisecs

////////////////////////////////////////////////
// Pins
////////////////////////////////////////////////

#define CAN_HW_ENABLE_PIN      10
#define ONE_WIRE_PIN           4
#define HEARTBEAT_LED_PIN      7

#define RELAY_ENABLE_PIN       3

////////////////////////////////////////////////
// Global Variables
////////////////////////////////////////////////

bool strobea=0; //var for heartbeat

float batt_avr_temp[3] = { 0.0f, 0.0f, 0.0f };
float batt_max_temp[3] = { 0.0f, 0.0f, 0.0f };
int batt_num_max_temp[3] = { 0, 0, 0 };

float batt_temp_threshold = 50.0f;

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



long short_timer_last = 0;
long short_timer_period = SHORT_TIMER_PERIOD;
long long_timer_last = 0;
long long_timer_period = LONG_TIMER_PERIOD;

EightByteData eight_byte_data;

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
    
    node_id = DEVICE_NODE_ID;
    node_type = DEVICE_NODE_TYPE;
    
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
    pinMode(HEARTBEAT_LED_PIN, OUTPUT);
    
    //enable relay enable
    pinMode(RELAY_ENABLE_PIN, OUTPUT);
    
    //and set it high to open darlington
    digitalWrite(RELAY_ENABLE_PIN, HIGH);

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
        
        digitalWrite(RELAY_ENABLE_PIN, HIGH);
      
        //Update Heartbeat
        strobea = 1-strobea;
        digitalWrite(HEARTBEAT_LED_PIN, strobea);
        
        Serial.print("Requesting temperatures...");
        temp_sensors.requestTemperatures(); // Send the command to get temperatures
        Serial.println("DONE");
        
        for(int j=0; j<3; j++) {
          float max_temp = 0.0f; int num_max_temp = 0;
          float tot_temp = 0.0f; int num_temps = 0;
          
          for(int i=0; i<12; i++) {
            float tempC = temp_sensors.getTempC(batt_add[j][i]);
            if ((tempC != 85.0) & (tempC != -127.0)){
              tot_temp = tot_temp + tempC; num_temps++;
              if (tempC > max_temp) { max_temp = tempC; num_max_temp = i; }
            }
          }
          
          batt_avr_temp[j] = tot_temp / num_temps;
          batt_max_temp[j] = max_temp;
          batt_num_max_temp[j] = num_max_temp;
          
          Serial.print("Max: "); Serial.print(max_temp);
          Serial.print("C @ #"); Serial.println(num_max_temp);
          Serial.print("Avr: "); Serial.print(batt_avr_temp[j]);
          Serial.print(" of "); Serial.println(num_temps);
          
          if (batt_max_temp[j] > batt_temp_threshold) {
            //Battery too hot, disable input to darlington and open contactors
            digitalWrite(RELAY_ENABLE_PIN, LOW);
          }
          
        }
        
        // Send Messages
        DUEMCANMessage msg_out;
        msg_out.CommandId = DATA_TRANSMIT;
        msg_out.TargetId = global_id;
        msg_out.Flags = 0;
        
        msg_out.DataFieldId = FIELD_BATT_TEMP_1;
        msg_out.DataFieldData.f = batt_max_temp[0];
        duem_send_message(msg_out);
        msg_out.DataFieldId = FIELD_BATT_TEMP_2;
        msg_out.DataFieldData.f = batt_avr_temp[0];
        duem_send_message(msg_out);
        msg_out.DataFieldId = FIELD_BATT_TEMP_3;
        msg_out.DataFieldData.f = batt_max_temp[1];
        duem_send_message(msg_out);
        msg_out.DataFieldId = FIELD_BATT_TEMP_4;
        msg_out.DataFieldData.f = batt_avr_temp[1];
        duem_send_message(msg_out);
        msg_out.DataFieldId = FIELD_BATT_TEMP_5;
        msg_out.DataFieldData.f = batt_max_temp[2];
        duem_send_message(msg_out);
        msg_out.DataFieldId = FIELD_BATT_TEMP_6;
        msg_out.DataFieldData.f = batt_avr_temp[2];
        duem_send_message(msg_out);
          
        //reset last counter
        long_timer_last = millis();
    }
    
    
    ////////////////////////////////////////////////
    // CAN message handler
    ////////////////////////////////////////////////
    
    if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    {
        CAN.readMsgBufID(&message_id, &message_len, message_buf);
        
        if ((message_id >= SENSOR_NODE_ID_BASE) && (message_id < SENSOR_NODE_ID_BASE+SENSOR_NODE_ID_RANGE)) {
          DUEMCANMessage msg = duem_rcv_message(message_id, message_len, message_buf);
          
          
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
