#include <SPI.h>
#include "mcp_can.h"
#include "LedControl.h"
#include "can_consts.h"

////////////////////////////////////////////////
// Device Settings
////////////////////////////////////////////////

// Filter to set RXB0 to. When RXB0 gets a message the output pin is pulled high which should reset the microcontroller.
#define DEVICE_RESET_ID         MAIN_CONTROLLER_RST_ID

#define DEVICE_NODE_TYPE        MAIN_CONTROLLER

// Time intervals between triggers for the two inner loops
#define SHORT_TIMER_PERIOD      100    //millisecs
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

#define DISP_DATA_PIN          0
#define DISP_CLK_PIN           A3
#define DISP_CS_PIN            A4


////////////////////////////////////////////////
// Global Variables
////////////////////////////////////////////////

bool strobea=0; //var for heartbeat

// Outputs for Motor
float motor_set_speed = 0.0f;
float motor_set_current = 0.0f;
float bus_set_current = 0.95f;

float car_speed = 0.0f;


// Inputs from driver
float driver_set_speed = 0.0f; //Input from potentiometer
bool accel_pedal_pressed = 0;

bool ignition_set = 0;
int direction_set = 0;

// Variables for doing the two timer firings
long short_timer_last = 0;
long short_timer_period = SHORT_TIMER_PERIOD;
long long_timer_last = 0;
long long_timer_period = LONG_TIMER_PERIOD;

// This struct is a useful hack for converting between datatypes.
struct{
  float f[2];
  INT8U c[8];
  INT32U i[2];
} eight_byte_data;

// holders for CAn stuff
INT32U message_id;
INT8U message_len;
INT8U message_buf[8];

// Instancing the CAN object (Note: this doesn't mean it's set up)
MCP_CAN CAN(CAN_HW_ENABLE_PIN);
LedControl ddisplay = LedControl(DISP_DATA_PIN, DISP_CLK_PIN, DISP_CS_PIN);

////////////////////////////////////////////////

void setup()
{
  //Uncomment for Serial for debugging, but this breaks stuff on pin 0/1
    //Serial.begin(115200);

////////////////////////////////////////////////
// Set up CAN interface and initialise various sensors
////////////////////////////////////////////////

START_INIT:

    if(CAN_OK == CAN.begin(CAN_BAUD_RATE)) // init can bus : baudrate = 500k, RX buf pins on
    {
        //Serial.println("CAN BUS Shield init ok!");
    }
    else
    {
        //Serial.println("CAN BUS Shield init fail");
        delay(100);
        goto START_INIT;
    }
    
    // If you don't understand CAN masks and Filters then read up on the MCP2515 or CAN in general
    // Filters allow the MCP2515 to only accept messages with certain IDs
    // The mask masks the filters for each buffer so can be used to create more general filters (ie filter a range)
    // Mask 0 is for RXB0, Mask 1 is for RXB1
    // Filters 0-1 are for RXB0
    // Filters 2-5 are for RXB1
    
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
    
    //This is a function i wrote which turns on the two buffer output pins on the MCP2515
    //When a message enters RXB0 or RXB1 the corresponding pin on the MCP2515 is pulled high
    //Since the output pin of RXB0 is tied to reset of the MC this means we can reset through
    //  CAN even if the MC has locked up. Simply send a message that will be accepted by RXB0
    CAN.enableBufferPins();

    //set up 7segs
    ddisplay.shutdown(0,false);
    ddisplay.setIntensity(0,2);
    ddisplay.clearDisplay(0);

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
        
        //Put stuff that you want to reasonably often here.
        
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

        if (ignition_set) {
          // Send Velocity + Current Message
          eight_byte_data.f[0] = motor_set_speed; //low float
          eight_byte_data.f[1] = motor_set_current; //high float
          CAN.sendMsgBuf(DRIVER_CONTROLS_BASE+1, 0, 8, eight_byte_data.c);
        }

        //reset last counter
        short_timer_last = millis();
    }
    
    ////////////////////////////////////////////////
    // LONG LOOP
    ////////////////////////////////////////////////
    
    milliseconds = millis();
    if ( (milliseconds >= long_timer_last + long_timer_period) || (milliseconds < long_timer_last) ) {
      
        //Put stuff that you want to happen occasionally here.
        
        
        //Update Heartbeat
        strobea = 1-strobea;
        digitalWrite(HEARTBEAT_LED_PIN, strobea);
        
        /////////////////
        // Send Bus Current Message
        eight_byte_data.f[0] = 0.0f; //low float
        eight_byte_data.f[1] = bus_set_current; //high float
        
        CAN.sendMsgBuf(DRIVER_CONTROLS_BASE+2, 0, 8, eight_byte_data.c);
        /////////////////


        //if (ignition_set) {
        if (1) {
          int huns = int(car_speed/100.0)%10;
          int tens = int(car_speed/10.0)%10;
          int units = int(car_speed)%10;
          int tenths = int(car_speed*10)%10;
          if (huns) ddisplay.setDigit(0,3,huns,false);
          if (tens) ddisplay.setDigit(0,2,tens,false);
          ddisplay.setDigit(0,1,units,true);
          ddisplay.setDigit(0,0,tenths,false);
        } else {
          ddisplay.clearDisplay(0);
        }
        
        //reset last counter
        long_timer_last = millis();
    }
    
    
    ////////////////////////////////////////////////
    // CAN message handler
    ////////////////////////////////////////////////
    
    if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    {
        CAN.readMsgBufID(&message_id, &message_len, eight_byte_data.c);

         if (message_id == MOTOR_CONTROLLER_BASE+3) {
            //Set values
            car_speed = eight_byte_data.f[1]*3.6;
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
