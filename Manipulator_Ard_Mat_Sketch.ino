
/* Code for controlling a Robot arm (5DOF) by using a single potentiometer and the serial read command
  to loop through all the independent motors and hence achive some degree of control*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define min_pulse_width  500
#define max_pulse_width  2400
#define frequency        50


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


//defining the Motor outputs on the PCA9685 board
int motor_base = 0;
int motor_elbow = 1;
int motor_wrist = 2;
int motor_pivot = 3;
int motor_jaws = 4;

int position_default;

char matlab_input;

int mtr_degree[5];
int angle;

int number_of_motors = 5;
int number_of_positions = 5;

int motor_rotation;
int motor_angle;

//char toExtract[8];


void setup() {

  pwm.begin();
  pwm.setPWMFreq(frequency);


  Serial.begin(9600);

}//End of the setup statements


void loop() {
  while (Serial.available()> 0) {
     
    static int angle = 0;
    
       matlab_input = Serial.read();

       int count = 0;
       
       switch(matlab_input){
        case '0'...'9':
        angle = angle * 10 + matlab_input - '0';
        break;
        
        case 'b':
        moveMotorDeg(angle, motor_base);
        Serial.println(angle);
        Serial.println("base");
        angle = 0;
        break;

        case 'e':
        moveMotorDeg(angle, motor_elbow);
        Serial.println(angle);
        Serial.println("elbow");
        angle = 0;
        break;  
        
        case 'w':
        moveMotorDeg(angle, motor_wrist);
        Serial.println(angle);
        Serial.println("wrist");
        angle = 0;
        break;  
        
        case 'p':
        moveMotorDeg(angle, motor_pivot);
        Serial.println(angle);
        Serial.println("pivot");
        angle = 0;
        break;  
        
        case 'j':
        moveMotorDeg(angle, motor_jaws);
        Serial.println(angle);
        Serial.println("jaw");
        angle = 0;      
        break;
        
       }

  //}

      /*if((position_default >=0) && (position_default <=180)){
        mtr_degree = angle;
        moveMotorDeg(mtr_degree, motor_base);
      Serial.println(matlab_input);
      Serial.println(position_default);
      Serial.println(angle);
      }

      else{
        Serial.println("Out of range");
        Serial.println("Stopping");
        Serial.println("Goodbye");
      }*/
      //Serial.println("End");

      //Moving to default position

      //moveMotorDeg(angle, motor_base);
     // moveMotorDeg(mtr_degree, motor_base);
   // } 
  }
  
}//End of the Loop statements




void moveMotorDeg(int motor_degrees, int motor_output) {
  int pulse_wide, pulse_width;

  //Conversion of the angle to pulse width
  pulse_wide = map(motor_degrees, 0, 180, min_pulse_width, max_pulse_width);
  pulse_width = int(float(pulse_wide) / 1000000 * frequency * 4096);

  //Moving the motor to desired location
  pwm.setPWM(motor_output, 0, pulse_width);
  

}//End of the moveMotorDeg fn

//void set_angles(){
//}

