

// These constants won't change.  They're used to give names
// to the pins used:
const int analogPin1 = A0;  // Analog input pin that the potentiometer is attached to
const int analogPin2 = A1;
const int analogPin3 = A2;

const int currentRate = 1000; //milliseconds between current data
const int chargeRate = 1000*60; //milliseconds between charge data


// ADC config

const int readRez = 10;
const float maxADCVal = powf(2,readRez)-1 ;

//Current sensor information

float noCurrentVoltage = 2.5; //expected voltage when current is 0 
float maxVolt = 5.0;
float ampRange = 300.0;  //maximum positive ampage


float sensor1Value;
float sensor2Value;
float sensor3Value;

float sensor1avg;
float sensor2avg;
float sensor3avg;

float sensor1curr;
float sensor2curr;
float sensor3curr;


void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(115200); 
}

void loop() {
    unsigned long chargeTime = millis();
    float dCharge1 = 0;
    float dCharge2 = 0;
    float dCharge3 = 0;
    while (millis()-chargeTime< chargeRate || millis()<chargeTime) //handle rollover
    { 
        unsigned long currTime = millis();
        int samples=0;
        float sensor1Value = 0;        
        float sensor2Value = 0;
        float sensor3Value = 0;

        while (millis()-currTime<currentRate || millis()<currTime ) //handle rollover
        {
            sensor1Value += analogRead(analogPin1);
            sensor2Value += analogRead(analogPin2);
            sensor3Value += analogRead(analogPin3);
            samples++;
            delay(2); //sleep 2 milliseconds to allow ADC to recover
        }
        sensor1avg = sensor1Value/float(samples);
        sensor2avg = sensor2Value/float(samples);
        sensor3avg = sensor3Value/float(samples);
        
        sensor1curr = ampRange*((sensor1avg/maxADCVal)-noCurrentVoltage/maxVolt);
        sensor2curr = ampRange*((sensor2avg/maxADCVal)-noCurrentVoltage/maxVolt);
        sensor3curr = ampRange*((sensor3avg/maxADCVal)-noCurrentVoltage/maxVolt);
        Serial.print(sensor1curr);
        Serial.print("\t");
        Serial.print(sensor2curr);
        Serial.print("\t");
        Serial.println(sensor3curr); //send CAN message about current here
        dCharge1 += sensor1curr;
        dCharge2 += sensor2curr;
        dCharge3 += sensor3curr;
        
    }
        Serial.print(dCharge1);
        Serial.print("\t");
        Serial.print(dCharge2);
        Serial.print("\t");
        Serial.println(dCharge3); //send CAN message about charge here
}
