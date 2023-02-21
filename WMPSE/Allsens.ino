#include<LiquidCrystal.h>
#include <Wire.h>
#include<LiquidCrystal.h>
#include "MAX30105.h"
#include "heartRate.h"
#define led1 7                     // for red led (warning)
#define led2 11                    // for green (no danger)
#define buzz 12                    // for buzzer (Warning)
float volt = 0;                    // LM35
float cel = 0;                     // LM35
float val1 = 0;                    // LM35
int sound = 600;                   // buzzer
int sensorValue;                   //MQ135
float sensor_volt;                 // MQ9
float RS_gas;                      //  Rs in clean air  MQ9
                                   
float sensValue = 0;               // MQ9
LiquidCrystal lcd(8, 9, 2, 3, 4, 5);
// MAX30102 variables

MAX30105 particleSensor;           // MAX30102 variables
const byte RATE_SIZE = 4;          //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];             //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0;                 //Time at which the last beat occurred
float beatsPerMinute=0;
int beatAvg =0; 
 long irValue =0;  
 long delta=0;                 
// end of variables for MAX30102

const int input = A0;              // LM35 Sensor input
void setup() {

  pinMode(led1, OUTPUT);           //danger led
  pinMode(led2, OUTPUT);           // no danger led
  //pinMode (pin no,input/output) to take output from that pin
  
  Serial.begin(115200);
  Serial.println("Initializing...");
                     // Initialize sensor MAX30102
                     
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  // End of Initializing

  lcd.begin(16, 2);
  lcd.setCursor(0, 1);             // for setting cursor
  lcd.print("Temp   Gas Conc.");   // for printing
  delay(4000);
  lcd.clear();
}

void loop() {
                                 // MAX30102 Start
                              
   irValue = particleSensor.getIR();
//long times=millis();
  if (checkForBeat(irValue) == true) {
    //We sensed a beat!
    delta =millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable
      
      //Take average of readings
      //beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
                                  // max30102 end
  
  //MQ135 start
  sensorValue = analogRead(1);             // read analog input pin 1 for MQ135
  //Serial.print("AirQua=");
 // Serial.print(sensorValue, DEC);          // prints the value read
  //Serial.println(" PPM");
  //MQ135 End
  
  //Serial.print("sensor_volt = ");          // delete this part latter MQ9
 // Serial.print(sensor_volt);
  //Serial.println("V");
 // Serial.print("R0 = ");
 // Serial.println(0.56);                    // MQ9
  
              //LM35 sensing start

              
  val1 = analogRead(input);                // reading from LM35 sensor for temp
 // volt = (val1 * 5000) / 1023.0;
 // cel = volt / 10;                                
   cel=val1/2.048;                         // temperature in celcius     end temp sensing
               // LM 35 End


                // MQ9 Start
float R0=0.91;
float ratio=0;
  //Average
  sensValue = analogRead(A2);
  sensor_volt = (sensValue / 1024) * 5.0;
  RS_gas = (5.0 - sensor_volt) / sensor_volt;      // Depend on RL on yor module
  ratio= RS_gas / 0.56;        //ratio fresh air   to present air  According to MQ9 datasheet table   end of MQ9 sensing
                               //MQ9 End
                               
                               

  // on serial display
   Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);
  Serial.print("  ");
  Serial.print(millis());
Serial.println("");
  if (irValue < 50000){
    Serial.print(" No finger?");
    lcd.print("No finger");
    delay(1000);
    lcd.clear();}

                               
                               // printing or displaying on lcd
  lcd.setCursor(0, 0);
  lcd.print("ArQ=");
  lcd.print(sensorValue, DEC);
  lcd.print("PPM");
  delay(2800);
  lcd.clear();
  lcd.print("RS ratio =");
  lcd.print(RS_gas);
  lcd.setCursor(0, 1);
  lcd.print("ratio =");
  lcd.print(ratio);
 delay(2800);
  lcd.clear();
  
  //printing temp on LCD
  lcd.setCursor(3, 0); 
  lcd.print("Temp= ");
  lcd.print(cel);
  lcd.print("  c");
  // end of temp
  
  delay(2800);
  lcd.clear();
  // HR display
  lcd.setCursor(1,0);
  lcd.print("BPM =");
  lcd.print(beatsPerMinute);
  lcd.setCursor(0,1);
  lcd.print("Avg BPM =");
  lcd.print(beatAvg);
  //end of hr display
  delay(2800);
  lcd.clear();
  
                   // for hazard condition test
  if (cel < 20 || cel > 60) {
    digitalWrite(led1, HIGH);
    digitalWrite(led2, LOW);
    tone(buzz, sound);
  }
  else {
    digitalWrite(led2, HIGH);
    digitalWrite(led1, LOW);
    noTone(buzz);
  }
                          // Ending test
  //delay(500);          // time for next reading
}
