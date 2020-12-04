
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

//Variables for Gyroscope
int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
boolean set_gyro_angles;

long acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc;

float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
float angle_pitch_output, angle_roll_output;

// Setup timers and temp variables
long loop_timer;
int temp;

// Display counter
int displaycount = 0;



void setup() {

  pwm.begin();
  pwm.setPWMFreq(frequency);

    //Start I2C
  Wire.begin();

    //Setup the registers of the MPU-6050                                                       
  setup_mpu_6050_registers(); 
  
  //Read the raw acc and gyro data from the MPU-6050 1000 times                                          
  for (int cal_int = 0; cal_int < 1000 ; cal_int ++){                  
    read_mpu_6050_data(); 
    //Add the gyro x offset to the gyro_x_cal variable                                            
    gyro_x_cal += gyro_x;
    //Add the gyro y offset to the gyro_y_cal variable                                              
    gyro_y_cal += gyro_y; 
    //Add the gyro z offset to the gyro_z_cal variable                                             
    gyro_z_cal += gyro_z; 
    //Delay 3us to have 250Hz for-loop                                             
    delay(3);                                                          
  }

  // Divide all results by 1000 to get average offset
  gyro_x_cal /= 1000;                                                 
  gyro_y_cal /= 1000;                                                 
  gyro_z_cal /= 1000;
  
  // Start Serial Monitor                                                 
  Serial.begin(9600);
  //Serial.begin(115200);
  
  
  // Init Timer 
  loop_timer = micros(); 

}//End of the setup statements


void loop() {

    // Get data from MPU-6050
  read_mpu_6050_data();
     
  //Subtract the offset values from the raw gyro values
  gyro_x -= gyro_x_cal;                                                
  gyro_y -= gyro_y_cal;                                                
  gyro_z -= gyro_z_cal;                                                  

  x_gyro = gyro_x;
  y_gyro = gyro_y;
  z_gyro = gyro_z;
  
  if (Serial.available()) {
     
    static int angle = 0;
    
       matlab_input = Serial.read();

       int count = 0;
       
       switch(matlab_input){
        case '0'...'9':
        angle = angle * 10 + matlab_input - '0';
        break;
        
        case 'b':
        moveMotorDeg(angle, motor_base);
        delay(500);
        //Serial.println(angle);
        //Serial.println("base");
        angle = 0;
        break;

        case 'e':
        moveMotorDeg(angle, motor_elbow);
        delay(500);
        //Serial.println(angle);
        //Serial.println("elbow");
        angle = 0;
        break;  
        
        case 'w':
        moveMotorDeg(angle, motor_wrist);
        delay(500);
        //Serial.println(angle);
        //Serial.println("wrist");
        
        angle = 0;
        break;  
        
        case 'p':
        moveMotorDeg(angle, motor_pivot);
        delay(500);
        //Serial.println(angle);
        //Serial.println("pivot");
        
        angle = 0;
        break;  
        
        case 'j':
        moveMotorDeg(angle, motor_jaws);
        delay(500);
        //Serial.println(angle);
        //Serial.println("jaw");
        angle = 0;        
  
        break;
        
       }
      
    for (int i=0; i<1; i++){  
      
        Serial.println(gyro_x);
        Serial.println(gyro_y);
        Serial.println(gyro_z);
    }
  
  }
  
}//End of the Loop statements


void setup_mpu_6050_registers(){

  //Activate the MPU-6050
  
  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68); 
  //Send the requested starting register                                       
  Wire.write(0x6B);  
  //Set the requested starting register                                                  
  Wire.write(0x00);
  //End the transmission                                                    
  Wire.endTransmission(); 
                                              
  //Configure the accelerometer (+/-8g)
  
  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68); 
  //Send the requested starting register                                       
  Wire.write(0x1C);   
  //Set the requested starting register                                                 
  Wire.write(0x10); 
  //End the transmission                                                   
  Wire.endTransmission(); 
                                              
  //Configure the gyro (500dps full scale)
  
  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68);
  //Send the requested starting register                                        
  Wire.write(0x1B);
  //Set the requested starting register                                                    
  Wire.write(0x08); 
  //End the transmission                                                  
  Wire.endTransmission(); 
                                              
}


void read_mpu_6050_data(){ 

  //Read the raw gyro and accelerometer data

  //Start communicating with the MPU-6050                                          
  Wire.beginTransmission(0x68);  
  //Send the requested starting register                                      
  Wire.write(0x3B);
  //End the transmission                                                    
  Wire.endTransmission(); 
  //Request 14 bytes from the MPU-6050                                  
  Wire.requestFrom(0x68,14);    
  //Wait until all the bytes are received                                       
  while(Wire.available() < 14);
  
  //Following statements left shift 8 bits, then bitwise OR.  
  //Turns two 8-bit values into one 16-bit value                                       
  acc_x = Wire.read()<<8|Wire.read();                                  
  acc_y = Wire.read()<<8|Wire.read();                                  
  acc_z = Wire.read()<<8|Wire.read();                                  
  temp = Wire.read()<<8|Wire.read();                                   
  gyro_x = Wire.read()<<8|Wire.read();                                 
  gyro_y = Wire.read()<<8|Wire.read();                                 
  gyro_z = Wire.read()<<8|Wire.read();                                 
}


void moveMotorDeg(int motor_degrees, int motor_output) {
  int pulse_wide, pulse_width;

  //Conversion of the angle to pulse width
  pulse_wide = map(motor_degrees, 0, 180, min_pulse_width, max_pulse_width);
  pulse_width = int(float(pulse_wide) / 1000000 * frequency * 4096);

  //Moving the motor to desired location
  pwm.setPWM(motor_output, 0, pulse_width);
  
}//End of the moveMotorDeg fn


