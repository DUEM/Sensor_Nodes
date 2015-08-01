#include <SPI.h>
#include "mcp_can.h"
#include "can_protocol.h"
#include "DUEM_can.h"

////////////////////////////////////////////////
// Device Settings
////////////////////////////////////////////////

#define DEVICE_NODE_ID          MAIN_CONTROLLER_ID
#define DEVICE_RESET_ID         DEVICE_NODE_ID + SENSOR_NODE_ID_RANGE

#define DEVICE_NODE_TYPE        MAIN_CONTROLLER

#define SHORT_TIMER_PERIOD      50     //millisecs
#define LONG_TIMER_PERIOD       1000   //millisecs

////////////////////////////////////////////////
// Pins
////////////////////////////////////////////////

#define CAN_HW_ENABLE_PIN      10
#define HEARTBEAT_LED_PIN      7

#define CRUISE_CONTROL_PIN     A5
#define ACCELERATOR_PIN        4
#define FORWARDS_PIN           3
#define REVERSE_PIN            2

////////////////////////////////////////////////
// Global Variables
////////////////////////////////////////////////

bool strobea=0; //var for heartbeat

// Outputs for Motor
float motor_set_speed = 0.0f;
float motor_set_current = 0.0f;
float bus_set_current = 0.0f;

// Inputs from driver
float driver_set_speed = 0.0f; //Input from potentiometer
bool accel_pedal_pressed = 0;
bool forwards_switch_pressed = 0;
bool reverse_switch_pressed = 0;

bool ignition_set = 0;
bool reverse_set = 0;

long short_timer_last = 0;
long short_timer_period = SHORT_TIMER_PERIOD;
long long_timer_last = 0;
long long_timer_period = LONG_TIMER_PERIOD;

EightByteData eight_byte_data;

MCP_CAN CAN(CAN_HW_ENABLE_PIN);

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
    CAN.init_Filt(4, 0, MOTOR_CONTROL_ID_BASE);
    CAN.init_Filt(5, 0, MOTOR_CONTROL_ID_BASE);
    
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
    
    //enable switch inputs
    pinMode(ACCELERATOR_PIN, INPUT_PULLUP);
    pinMode(FORWARDS_PIN, INPUT_PULLUP);
    pinMode(REVERSE_PIN, INPUT_PULLUP);    

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
        
        //INPUTS WILL BE LOW WHEN BUTTON IS PRESSED
        int input_a = digitalRead(ACCELERATOR_PIN);
        int input_b = digitalRead(FORWARDS_PIN);
        int input_c = digitalRead(REVERSE_PIN);
        
        if (input_b == LOW) {
          forwards_switch_pressed = 1; reverse_set = 0; ignition_set = 1;
        } else if (input_c == LOW) {
          reverse_switch_pressed = 1; reverse_set = 1; ignition_set = 1;
        } else { forwards_switch_pressed = 0; reverse_switch_pressed = 0; ignition_set = 0; reverse_set = 0; }
        
        if (input_a = LOW) {
          accel_pedal_pressed = 1;
          motor_set_current = 0.75;
        } else {
          accel_pedal_pressed = 0;
          motor_set_current = 0.0;          
        }
        
        if (ignition_set) {
          // Send Velocity + Current Message
          eight_byte_data.f[0] = motor_set_speed; //low float
          eight_byte_data.f[1] = motor_set_current; //high float
          
          CAN.sendMsgBuf(DRIVER_CONTROL_ID_BASE+1, 0, 8, eight_byte_data.c);
        }

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
        
        /////////////////
        // Send Bus Current Message
        eight_byte_data.f[0] = 0.0f; //low float
        eight_byte_data.f[1] = bus_set_current; //high float
        
        CAN.sendMsgBuf(DRIVER_CONTROL_ID_BASE+2, 0, 8, eight_byte_data.c);
        /////////////////
        
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
          
          
        } else if ((message_id >= MOTOR_CONTROL_ID_BASE) && (message_id < MOTOR_CONTROL_ID_BASE+MOTOR_CONTROL_ID_RANGE)) {
          //Handle Motor Controller Messages
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
