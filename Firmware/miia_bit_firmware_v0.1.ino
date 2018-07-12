// LOFI Brain firmware to communicate with LOFI Robot ScratchX Chrome Plugin
// USB + Bluetooth version
// Author: Maciej Wojnicki
// WWW.LOFIROBOT.COM

/*
 * Lofi firmware developed for MiiA.bit 
 * Version v0.1
 * Date: 10/07/2018
 * Instructions: 
 * Connect the control board to your computer using the specified USB cable.
 * Set the board to the arduino board, in the arduino ide go to:
 * tools -> board -> arduino Nano
 * Connect port:
 * tools -> port -> choose port board is connected to
 * click the upload button(use the second button from the left above, arrow pointing right)
 * ******************************************************************
 * Electronic Connections:
 * ******************************************************************
 * While facing the front of MiiA.bit:
 * 
 * 
 */

// included libraries
#include <Servo.h>

// function prototypes
void set_servo_position(Servo my_servo, int pin, int pos);
void servo_off(Servo my_servo);
void configure_pins();
void initialize_pins();
void motor_control(int motor_forward_pin, int motor_reverse_pin, int speed);
void initialization_ok();
void sound_buzzer(int frequency);
long read_distance_uss();
void receiving();
void outputs_set();
void sending();

// pin assignments
// servo motor
#define servo_arm_pin 3 // 1

// ultrasonic sensor
#define uss_trigger_pin 17 // 26
#define uss_echo_pin 18 // 27

// RBG LED
#define led_r_pin 14 // 23
#define led_g_pin 15 // 24
#define led_b_pin 16 // 25

// DC motors
#define motor_a_forward_pin 5
#define motor_a_reverse_pin 6
#define motor_b_forward_pin 13
#define motor_b_reverse_pin 14

// buzzer
#define buzzer_pin A6 // 20 //19 

// push button
#define push_button_pin 19

// constants
#define baud_rate 57600 // nb baud rate must be 57600 for LoFi Blocks

#define servo_calibration_pos 90

// create servo objects
Servo servo_arm;

//data sending (arduino->computer) interval  
//raise it if you encouter communication jitter
const long interval = 10000;

int current_byte = 0;
int prev_byte = 0;
int analog1 = 1;
int analog2 = 2;
int analog3 = 3;
int analog4 = 4;

unsigned long previousMillis = 0;
unsigned long currentMillis;

void setup()
{

  // configure all pin modes
  configure_pins();

  // board initialization - ok!
  initialization_ok();

  // set calibration pin and drive pin to calibrated position
  set_servo_position(servo_arm, servo_arm_pin, servo_calibration_pos);

  // initialize serial communications at specified baud rate
  Serial.begin(baud_rate);

  // setup complete
}

void loop()
{
  currentMillis = millis();

  //receiving data from ScratchX Chrome plugin
  receiving();

  // timer delay reduce data bandwidth
  if (currentMillis - previousMillis >= interval) 
  {
    
    previousMillis = currentMillis;

    //sending data to ScratchX Chrome plugin
    sending();
  }
}

// board initialization - ok!: led blue flash twice
void initialization_ok()
{
  analogWrite(led_b_pin, 0);
  delay(500);
  analogWrite(led_b_pin, 255);
  delay(250);
  analogWrite(led_b_pin, 0);
  delay(500)
  analogWrite(led_b_pin, 255);
}

// ultrasonic sensor function
long read_distance_uss()
{
  
  long duration, distance;
  digitalWrite(uss_trigger_pin, LOW); 
  delayMicroseconds(2); 
  digitalWrite(uss_trigger_pin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(uss_trigger_pin, LOW);
  duration = pulseIn(uss_echo_pin, HIGH);
  distance = (duration/2)/29.1;
  return distance;  
}

// function to receive serial data from LoFi program
void receiving()
{
  if (Serial.available() > 0)
  {
  current_byte = Serial.read();
  outputs_set();
  prev_byte = current_byte;
  } 
}

// function to perform output based on LoFi program
void outputs_set()
{   
  // buzzer on/off set tone (Buzzer on LoFi)
  if (prev_byte == 201)
  {
    // turn the buzzer on
    if (current_byte == 1)
    {
      tone(buzzer_pin, 2400);
    }

    // turn the buzzer off
    if (current_byte == 0)
    {
      noTone(buzzer_pin);
    }
  }

  // motor a (M1 on LoFi)
  if (prev_byte == 202)  
  {
    motor_control(motor_a_forward_pin, motor_a_reverse_pin, current_byte);
  }

  // motor b (M2 on LoFi)
  if (prev_byte == 203) 
  {
    motor_control(motor_b_forward_pin, motor_b_reverse_pin, current_byte);
  }

  // Rgb output (LoFi output 1)
  if (prev_byte == 204)
  {
      analogWrite(led_r_pin, 255-(current_byte*2.55));
  }
  // rGb output (LoFi output 2)
  if (prev_byte == 205)
  {
      analogWrite(led_g_pin, 255-(current_byte*2.55));
  }
  // rgB output (LoFi output 3)
  if (prev_byte == 206)
  {
      analogWrite(led_b_pin, 255-(current_byte*2.55));
  }

  // buzzer - custom tone (LoFi output 4)
  if (prev_byte == 207)
  {
    sound_buzzer(current_byte);
  }

  // servo arm output (LoFi servo output 1)
  if (prev_byte == 208)
  {
      set_servo_position(servo_arm, servo_arm_pin, current_byte*1.8);
  }

  // servos off
  if (prev_byte == 212 && current_byte == 99)
  {
    // detach all servos
    servo_off(servo_arm);
  }
}

// function to send information to LoFi program
void sending()
{
  // even if not using analog inputs, LoFi requires this.
  Serial.write(224);
  Serial.write(byte(analog1));
  Serial.write(225);
  Serial.write(byte(analog2));
  Serial.write(226);
  Serial.write(byte(analog3));
  Serial.write(227);
  Serial.write(byte(analog4));
  Serial.write(240);

  // ultra sonic sensor output
  Serial.write(byte(read_distance_uss()));
  // last byte "i" character as a delimiter for BT2.0 on Android
  Serial.write(105);
}

// function to configure pins and pin modes
void configure_pins()
{
  
  // miia.bit servo motor signal pins
  pinMode(servo_arm_pin, OUTPUT);

  // ultrasonic sensor trigger and echo pins
  pinMode(uss_trigger_pin, OUTPUT);
  pinMode(uss_echo_pin, INPUT);

  // dc motor pins
  pinMode(motor_a_forward_pin, OUTPUT);
  pinMode(motor_a_reverse_pin, OUTPUT);
  pinMode(motor_b_forward_pin, OUTPUT);
  pinMode(motor_b_reverse_pin, OUTPUT);

  // RGB LED pins
  pinMode(led_r_pin, OUTPUT);
  pinMode(led_g_pin, OUTPUT);
  pinMode(led_b_pin, OUTPUT);

  // buzzer
  pinMode(buzzer_pin, OUTPUT);
}

// function to set the default pin states
void initialize_pins()
{
  // turn all LEDs off
  analogWrite(led_r_pin, 255);
  analogWrite(led_g_pin, 255);
  analogWrite(led_b_pin, 255);

  // turn buzzer off

  // turn motors off
  analogWrite(motor_a_forward_pin, 0);
  analogWrite(motor_a_reverse_pin, 0);
  analogWrite(motor_b_forward_pin, 0);
  analogWrite(motor_b_reverse_pin, 0);

  // reset servo to calibrated position
  set_servo_position(servo_arm, servo_arm_pin, servo_calibration_pos);

}

// function to turn off, and detach a servo
void servo_off(Servo my_servo)
{
  my_servo.detach();
}

// function to set position of a servo motor
void set_servo_position(Servo my_servo, int pin, int pos)
{
  my_servo.attach(pin);
  my_servo.write(pos);
}

// function to control a dc motor
void motor_control(int motor_forward_pin, int motor_reverse_pin, int speed)
{
    // forward direction
    if (speed <= 100) 
    {
      // current_byte is the % power
      analogWrite(motor_forward_pin, speed*2.55);
      analogWrite(motor_reverse_pin, 0);
      
    }
    // reverse direction
    if (speed > 100) 
    {
      // current_byte is the % power
      analogWrite(motor_forward_pin, 0);
      analogWrite(motor_reverse_pin, (speed-100)*2.55);
    }
}

// function to control a buzzer
void sound_buzzer(int frequency)
{

}

//end
