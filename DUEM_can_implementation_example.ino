#include <SPI.h>
#include "mcp_can.h"
#include "can_protocol.h"
#include "DUEM_can.h"

// can_protocol contains the specific defines for the current protocol
// DUEM_can handles DUEM's deprecated message wrapper and some other functions

////////////////////////////////////////////////
// Device Settings
////////////////////////////////////////////////

#define DEVICE_NODE_ID          MAIN_CONTROLLER_ID
#define DEVICE_RESET_ID         DEVICE_NODE_ID + SENSOR_NODE_ID_RANGE

#define DEVICE_NODE_TYPE        MAIN_CONTROLLER

#define SHORT_TIMER_PERIOD      100     //millisecs
#define LONG_TIMER_PERIOD       1000   //millisecs

////////////////////////////////////////////////
// Pins
////////////////////////////////////////////////

#define CAN_HW_ENABLE_PIN      10
#define HEARTBEAT_LED_PIN      7

#define SPEED_SET_PIN          A5
#define ENABLE_PIN             1
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
float bus_set_current = 0.95f;


// Inputs from driver
float driver_set_speed = 0.0f; //Input from potentiometer
bool accel_pedal_pressed = 0;

bool ignition_set = 0;
int direction_set = 0;

// These are timers and stuff for doing loops which fire every X milliseconds
long short_timer_last = 0;
long short_timer_period = SHORT_TIMER_PERIOD;
long long_timer_last = 0;
long long_timer_period = LONG_TIMER_PERIOD;

// instancing the eightbytedata which is a hack used to change between floats and bytes
EightByteData eight_byte_data;

// settign up a CAN instance, the argument taken is the device enable pin for spi (generally pin 10)
MCP_CAN CAN(CAN_HW_ENABLE_PIN);

////////////////////////////////////////////////

void setup()
{
    //Serial.begin(115200);

////////////////////////////////////////////////
// Set up CAN interface and initialise various sensors
////////////////////////////////////////////////

START_INIT:

    //This loop here tries to establish communication with the MCP2515
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
    
    //Setting up the Mask and Filter for each of the MCP2515's RX buffers (read up on it) using the Seeed library
    //Mask 0 is for RXB0, Mask 1 is for RXB1
    //Filter 0-1 are for RXB0, Filter 2-5 are for RXB1 (crazy, i know)
    
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
    
    //CAN.init_Filt(4, 0, CAN_DEFAULT_FILTER);
    //CAN.init_Filt(5, 0, CAN_DEFAULT_FILTER);
    
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
    pinMode(8, OUTPUT);
    
    //enable switch inputs
    pinMode(ACCELERATOR_PIN, INPUT_PULLUP);
    pinMode(ENABLE_PIN, INPUT_PULLUP);
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
        int input_d = digitalRead(ENABLE_PIN);
        int input_e = analogRead(SPEED_SET_PIN);
        
        if (input_a == LOW) {
          digitalWrite(8, HIGH);
          motor_set_current = 0.75;
        } else {
          motor_set_current = 0.0;
          digitalWrite(8, LOW);        
        }
        
        if (input_b == LOW) {
          direction_set = 1;
          digitalWrite(8, HIGH);
        } else if (input_c ==LOW) {
          direction_set = -1;
        } else {
          direction_set = 0;
          digitalWrite(8, LOW);  
        }
        
        if (input_d == LOW) {
          ignition_set = 1;
        } else {
          ignition_set = 0;
        }
        
        driver_set_speed = input_e / 10.24;
        motor_set_speed = driver_set_speed * direction_set;
        
        //long_timer_period = 2000;
        
        if (ignition_set) {
          
          //long_timer_period = 1100 - (motor_set_speed * 10);
          
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
