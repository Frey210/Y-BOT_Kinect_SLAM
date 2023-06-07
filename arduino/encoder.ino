//void setup () {

//serial.begin (9600);

// Constants
const int rencoderPinA = 2;    // Right Encoder pin A connected to digital pin 2
const int rencoderPinB = 3;    // Right Encoder pin B connected to digital pin 3
const int lencoderPinA = 20;    // Left Encoder pin A connected to digital pin 20
const int lencoderPinB = 21;    // Left Encoder pin B connected to digital pin 21

// Variables
volatile long rencoderCount = 0;  // Current encoder count
volatile int rencoderState = 0;   // Current encoder state
int previousState_rencoder = 0;   // Previous encoder state
volatile long lencoderCount = 0;  // Current encoder count
volatile int lencoderState = 0;   // Current encoder state
int previousState_lencoder = 0;   // Previous encoder state
// Interrupt service routine for encoder pin A
void encoderISR_rencoder()
{
  // Read the state of encoder pin B
  int pinBState = digitalRead(rencoderPinB);

  // Update the encoder count based on the direction
  if (pinBState == HIGH)
    rencoderCount++;
  else
    rencoderCount--;
}
void encoderISR_lencoder()
{
  // Read the state of lencoder pin B
  int pinBState = digitalRead(lencoderPinB);

  // Update the encoder count based on the direction
  if (pinBState == HIGH)
    lencoderCount++;
  else
    lencoderCount--;
}
void setup()
{
  // Initialize encoder pins as inputs
  pinMode (rencoderPinA, INPUT_PULLUP);
  pinMode (rencoderPinB, INPUT_PULLUP);
  pinMode (lencoderPinA, INPUT_PULLUP);
  pinMode (lencoderPinB, INPUT_PULLUP);

  // Enable internal pull-up resistors for the encoder pins
  digitalWrite(rencoderPinA, HIGH);
  digitalWrite(rencoderPinB, HIGH);

  // Attach the interrupt for encoder pin A
  attachInterrupt(digitalPinToInterrupt(rencoderPinA), encoderISR_rencoder, RISING);
  attachInterrupt(digitalPinToInterrupt(lencoderPinA), encoderISR_lencoder, CHANGE);

  // Start serial communication
  Serial.begin(9600);
}

void loop()
{
  if (lencoderCount > 100 || rencoderCount > 100) {
    rencoderCount = 100;
    lencoderCount = 100;
  }
  Serial.println(lencoderCount);
  Serial.println(rencoderCount);

  delay(500);
}
