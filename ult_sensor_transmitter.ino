#include <RH_ASK.h>
#include <SPI.h> // Not actually used but needed to compile
//datapin must be arduino digital pin 12
RH_ASK driver;

//ultrasensor input config
#define ECHO_PIN 7
#define TRIG_PIN 5
#define AVG_SAMPLES 20
#define LEVER_ERROR_UPPER_LIM 75.00
#define ERROR_LAMP_PIN 13

double tr_data = 99.99;
long duration; // variable for the duration of sound wave travel
double distance;



double measureLevel() {
  // Clears the trigPin condition
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(ECHO_PIN, HIGH);
  // Calculating the distance
  distance = (double)(duration * 0.034 / 2); // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
 // Serial.print("Distance: ");
  //Serial.print(distance);
 // Serial.println(" cm");
  return distance;
}


void setup()
{
  Serial.begin(9600);    // Debugging only
  pinMode(TRIG_PIN, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(ECHO_PIN, INPUT);
  pinMode(ERROR_LAMP_PIN, OUTPUT);
  if (!driver.init())
    Serial.println("init failed");
}

void loop()
{
  static double sumForLvlAvg;
  static uint8_t avgSamplCtr;
  tr_data = 0.0;
  sumForLvlAvg += measureLevel();
  avgSamplCtr++;

  if (avgSamplCtr == AVG_SAMPLES) {
    tr_data = (double)(sumForLvlAvg / AVG_SAMPLES);
     
    if(tr_data>LEVER_ERROR_UPPER_LIM){
          digitalWrite(ERROR_LAMP_PIN,HIGH);
          delay(500);
          digitalWrite(ERROR_LAMP_PIN,LOW);
          delay(500); 
      }
      else{
        Serial.println(tr_data);
          driver.send((uint8_t *)&tr_data, 4);
          driver.waitPacketSent();
        }
    
    avgSamplCtr = 0;
    sumForLvlAvg = 0;
  }
 

  delay(100);
}
