#include <OneWire.h>
#include <DallasTemperature.h>
 
// Data wire is plugged into pin 8 on the Arduino
#define ONE_WIRE_BUS 8

// Relay control output pins
#define PV_RELAYS 2
#define MOTOR_RELAYS 3
#define LV_RELAYS 4

// Maximum temperature is set here
#define MAX_TEMP 40

// Setup a oneWire instance to communicate with any OneWire devices 
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
 
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
 
void setup(void)
{
  // start serial port
  Serial.begin(9600);
  // Start up the library
  sensors.begin();
  // Serial Only used for Debugging Purposes

  pinMode(PV_RELAYS,OUTPUT);
  pinMode(MOTOR_RELAYS,OUTPUT);
  pinMode(LV_RELAYS,OUTPUT);

  //Switch relays on intially
  digitalWrite(PV_RELAYS, HIGH);
  digitalWrite(MOTOR_RELAYS, HIGH);
  digitalWrite(LV_RELAYS, HIGH); 
}
 
 
void loop(void)
{
  int SensorsConnected = 2; 
  int Sensor;   
  //scanSensors(); // Detects number of sensors and sets global variable maxsensors
  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  //Serial.print(" Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  //Serial.println("DONE");
  /*************************************************************/
  for(Sensor = 0; Sensor< SensorsConnected; Sensor++){
    Serial.print(sensors.getTempCByIndex(Sensor)); //Prints temperature to serial. Replace with CAN BUS output instead
    Serial.print("\n");
    if (sensors.getTempCByIndex(Sensor) >= MAX_TEMP){
        digitalWrite(PV_RELAYS, LOW);
        digitalWrite(MOTOR_RELAYS, LOW); // Switch Relays off if Batteries get too hot. 
        digitalWrite(LV_RELAYS, LOW); 
    }
  }
  /*************************************************************/

 
 
}
