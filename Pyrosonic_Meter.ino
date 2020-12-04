/*
            Pyrosonic Meter
This project attempts to model an automatic pyrometer operation. The components used are the ultrasonic sensor and the DHT11 temp sensor.
the mode of operation is such that the DHT displays the output temperature when an object distance of less than 30cm is achieved. the measure of distance is done by the ultrasonic sensor.
*/


#include <LiquidCrystal.h> //Icluding the LCD header file
#include <dht11.h>
#include <Servo.h>

#define trigPin 9
#define echoPin 8
#define dhtSensor 12
#define buzzer 13
int redLed = 10;
//int greenLed = 11;
int potpin = A0;  // analog pin used to connect the potentiometer

int val;    // variable to read the value from the analog pin


float duration, distance;

Servo myservo;
dht11 DHT11;
LiquidCrystal lcd(2, 3, 4, 5, 6, 7);//header file initialization

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(redLed, OUTPUT);
  //pinMode(greenLed, OUTPUT);
  pinMode(buzzer, OUTPUT);
  
  myservo.attach(11);  // attaches the servo on pin 9 to the servo object
  
  lcd.begin(16, 2);
  lcd.clear();
  Serial.begin(9600);
  
  
}

void loop() {
  int temp = DHT11.read(dhtSensor);
  digitalWrite(trigPin, LOW);
  delay(2000);
  digitalWrite(trigPin, HIGH); //Pulse transmission
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH); //Measure of the pulse width in mS
  distance = (duration / 2) * 0.0343; // Where the speed of sound is 343m/s in air
  
  lcd.clear();  
  if(distance <=400 && distance >=2){
    
    Serial.print("Distance: ");
    Serial.println(distance);
    
    Serial.print("Humidity: ");
    Serial.println((float)DHT11.humidity, 2);
    Serial.print("Temperature: ");
    Serial.println((float)DHT11.temperature, 2);
    Serial.println();
   
    lcd.println("Temp: ");
    lcd.println((float)DHT11.temperature, 2);
    
    lcd.setCursor(0,1);
    lcd.print("Humidity(%): ");
    lcd.println((float)DHT11.humidity, 2);
    
    digitalWrite(redLed, LOW);
    //digitalWrite(greenLed, HIGH);
    noTone(buzzer);
    
    val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023) 
    val = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180) 
    myservo.write(val);                  // sets the servo position according to the scaled value 
    delay(15); 
  
  }
  else{
    
    digitalWrite(redLed, HIGH);
    //digitalWrite(greenLed, LOW);
    Serial.print("The distance is: ");
    Serial.println("Out of range");
    lcd.println("Distance: ");
    lcd.println("Out of range");
    
    tone(buzzer, 2500, 500);
    delay(1000);
  
  }
  
}
