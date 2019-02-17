// LOFI Brain firmware to communicate with LOFI Robot ScratchX Chrome Plugin
// USB + Bluetooth version
// Author: Maciej Wojnicki
// WWW.LOFIROBOT.COM

/*
 * Lofi firmware developed for MiiA.bit
 * Version v0.3
 * Date: 04/09/2018
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

// function prototypes
void configure_pins();
void initialize_pins();
void motor_control(int motor_forward_pin, int motor_reverse_pin, int speed);
void initialization_ok();
long read_distance_uss();
void receiving();
void outputs_set();
void sending();
void servo_off(int servo_pin);
void set_servo_position(int servo_pin, int pos);
int dist;

// pin assignments
// servo motor
#define servo_arm_pin 3

// ultrasonic sensor
#define uss_trigger_pin 17
#define uss_echo_pin 18
int trigPin = 17;
int echoPin = 18;

// RBG LED
#define led_r_pin 14
#define led_g_pin 15 
#define led_b_pin 16 

// DC motors
#define motor_a_forward_pin 9
#define motor_a_reverse_pin 10
#define motor_b_forward_pin 5
#define motor_b_reverse_pin 6

// buzzer
#define buzzer_pin 8 

// push button
#define push_button_pin 19

// constants
#define baud_rate 57600 // nb baud rate must be 57600 for LoFi Blocks Chrome

#define servo_calibration_pos 90

int lenMicroSecondsOfPeriod = 25 * 1000; // 25 milliseconds (ms)
int lenMicroSecondsOfPulse = 1 * 1000; // 1 ms is 0 degrees
int min_pos = 1 * 1000; //0.5ms is 0 degrees in HS-422 servo
int max_pos = 2 * 1000;
int increment = 0.01 * 1000;
int current_pos = 0;

//data sending (arduino->computer) interval  
//raise it if you encouter communication jitter
const long interval = 10000;

int current_byte = 0;
int prev_byte = 0;
int analog1 = 0;
int analog2 = 0;
int analog3 = 0;
int analog4 = 0;

unsigned long previousMillis = 0;
unsigned long currentMillis;

void setup()
{

  // configure all pin modes
  configure_pins();

  // initialize all pins
  initialize_pins();

  // board initialization - ok!
  initialization_ok();

  // set calibration pin and drive pin to calibrated position
  set_servo_position(servo_arm_pin, servo_calibration_pos);

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
  analogWrite(led_r_pin, 255);
  analogWrite(led_g_pin, 255);
  analogWrite(led_b_pin, 0);
  delay(1000);
  analogWrite(led_r_pin, 255);
  analogWrite(led_g_pin, 255);
  analogWrite(led_b_pin, 255);
  delay(500);
  analogWrite(led_r_pin, 255);
  analogWrite(led_g_pin, 255);
  analogWrite(led_b_pin, 0);
  delay(1000);
  analogWrite(led_r_pin, 255);
  analogWrite(led_g_pin, 255);
  analogWrite(led_b_pin, 255);
}

// ultrasonic sensor function
long read_distance_uss()
{
  
  long duration, distance;
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);

  delayMicroseconds(5); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH, 5000);
  distance = (duration/2) / 29.1;
  
  dist = distance;
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
    //sound_buzzer(current_byte);
  }

  // servo arm output (LoFi servo output 1)
  if (prev_byte == 208)
  {
      // write current_byte*1.8 to servo
      set_servo_position(servo_arm_pin, current_byte);
  }

  // servos off
  if (prev_byte == 212 && current_byte == 99)
  {
    // put servo off
  }
}

// function to send information to LoFi program
void sending()
{
  // first read and send the analog inputs
  // even if not using analog inputs, LoFi requires this.

  // push button is Analog Input 1
  analog1 = analogRead(push_button_pin)/10.23;
  Serial.write(224);
  Serial.write(byte(analog1));
  
  // other analog inputs currently not used
  Serial.write(225);
  Serial.write(byte(analog2));
  Serial.write(226);
  Serial.write(byte(analog3));
  Serial.write(227);
  Serial.write(byte(analog4));
  Serial.write(240);

  // ultra sonic sensor output
  read_distance_uss();
  Serial.write(byte(dist));
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

  // button
  pinMode(push_button_pin, INPUT);
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
  set_servo_position(servo_arm_pin, servo_calibration_pos);

}

// function to set position of a servo motor
void set_servo_position(int servo_pin, int pos)
{
  int pos_setting = ((pos*1.8) * 5.50) + 1000;
  if (pos_setting > current_pos)
  {
    for(int pwm = current_pos; pwm <= pos_setting; pwm += increment)
    {
      digitalWrite(servo_pin, HIGH);
      delayMicroseconds(pwm);
      digitalWrite(servo_pin, LOW);
      delay(15);
      
    }
  }

  if (pos_setting < current_pos)
  {
    for(int pwm = current_pos; pwm >= pos_setting; pwm -= increment)
    {
      digitalWrite(servo_pin, HIGH);
      delayMicroseconds(pwm);
      digitalWrite(servo_pin, LOW);
      delay(15);
      
    }
  }
  current_pos = pos_setting;
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

//end
