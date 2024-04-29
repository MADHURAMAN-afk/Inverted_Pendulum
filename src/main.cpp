#include <AccelStepper.h>

// Define rotary encoder pins
#define ENC_A 2
#define ENC_B 3


void read_encoder();

unsigned long _lastIncReadTime = micros(); 
unsigned long _lastDecReadTime = micros(); 
int _pauseLength = 25000;
int _fastIncrement = 10;

volatile int counter = 0;

// Stepper motor pins
#define STEP_PIN 9
#define DIR_PIN 8

// Define stepper motor parameters
#define STEPS_PER_REVOLUTION 200 // Change according to your motor's specifications

// Create an AccelStepper object for controlling the stepper motor
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  // Set encoder pins and attach interrupts
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), read_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), read_encoder, CHANGE);

  // Set up stepper motor
  stepper.setMaxSpeed(1000); // Adjust speed as needed
  stepper.setAcceleration(500); // Adjust acceleration as needed

  // Start the serial monitor to show output
  Serial.begin(115200);
}

void loop() {
  // Move the stepper motor to the desired position based on the encoder input
  stepper.runToNewPosition(counter * STEPS_PER_REVOLUTION);

  // Print the current stepper motor position
  
  // Serial.println(stepper.currentPosition());
  static int lastCounter = 0;

  // If count has changed print the new value to serial
  if(counter != lastCounter){
    Serial.println(counter);
    lastCounter = counter;
  }
}

void read_encoder() {
  // Encoder interrupt routine for both pins. Updates counter
  // if they are valid and have rotated a full indent
 
  static uint8_t old_AB = 3;  // Lookup table index
  static int8_t encval = 0;   // Encoder value  
  static const int8_t enc_states[]  = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Lookup table

  old_AB <<=2;  // Remember previous state

  if (digitalRead(ENC_A)) old_AB |= 0x02; // Add current state of pin A
  if (digitalRead(ENC_B)) old_AB |= 0x01; // Add current state of pin B
  
  encval += enc_states[( old_AB & 0x0f )];

  // Update counter if encoder has rotated a full indent, that is at least 4 steps
  if( encval > 1 ) {        // Four steps forward
    int changevalue = 1;
    _lastIncReadTime = micros();
    counter += changevalue; // Update counter
    encval = 0;
  }
  else if( encval < -1 ) {  // Four steps backward
    int changevalue = -1;
    _lastDecReadTime = micros();
    counter += changevalue; // Update counter
    encval = 0;
  }
}

