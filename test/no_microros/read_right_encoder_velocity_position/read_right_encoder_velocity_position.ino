// Encoder pins
// A leads B for forward rotation
#define ENC_R_1 0 // Encoder 1, Channel A (YELLOW)
#define ENC_R_2 1 // Encoder 1, Channel B (WHITE)


const int PPR = 990;           // Pulses per revolution
const float WHEEL_RADIUS = 0.035; // 3.5 cm radius

// Variables for timing and velocity
volatile unsigned long last_time_right = 0;
volatile float vel_r = 0.0;

volatile int posi_r = 0;

void read_encoder_right() {
  unsigned long current_time = micros();
  unsigned long delta_time = current_time - last_time_right;
  last_time_right = current_time;

  if (delta_time > 0) {
      vel_r = (1.0 / delta_time) * (2.0 * PI / PPR) * 1e6; // Convert µs to seconds
  }
  // Read the state of encoder 1 channel B
  int b = digitalRead(ENC_R_2);
  
  // Increment or decrement the position of encoder 1 based on the state of channel B
  if (b > 0) {
    posi_r--;
  } else {
    posi_r++;
  }
}


void setup() {
  Serial.begin(9600);
  pinMode(ENC_R_1, INPUT_PULLUP);
  pinMode(ENC_R_2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_R_1), read_encoder_right, RISING);
}

void loop() {
  static unsigned long last_print_time = 0;
  unsigned long current_time = millis();
  
  if (current_time - last_print_time >= 5) {
      Serial.println(vel_r);
      last_print_time = current_time;
  }
}
