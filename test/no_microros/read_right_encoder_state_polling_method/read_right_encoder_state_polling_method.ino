// Encoder pins
#define ENC_R_1 0 // Encoder 1, Channel A (YELLOW)
#define ENC_R_2 1 // Encoder 1, Channel B (WHITE)

const int PPR = 990; // Pulses per revolution

// Global variable to store the angular velocity
volatile float ang_vel_r = 0.0; // Angular velocity in rad/s

// Position variable for the encoder
volatile int posi_r = 0;

// Interrupt Service Routine to update encoder position
void read_encoder_right() {
  // Read the state of encoder 1 channel B
  int b = digitalRead(ENC_R_2);

  // Increment or decrement the position based on the state of channel B
  if (b > 0) {
    posi_r--;
  } else {
    posi_r++;
  }
}

// Function to calculate angular velocity based on polling
void calculate_angular_velocity(unsigned long poll_period_ms) {
  static unsigned long last_poll_time = 0;
  static int last_posi_r = 0;

  unsigned long current_time = millis();

  // Check if polling period has elapsed
  if (current_time - last_poll_time >= poll_period_ms) {
    int current_posi_r = posi_r; // Get the latest position
    int delta_position = current_posi_r - last_posi_r; // Change in position
    last_posi_r = current_posi_r;

    // Calculate angular velocity (rad/s)
    ang_vel_r = (delta_position / (float)PPR) * (2 * PI) / (poll_period_ms / 1000.0);

    last_poll_time = current_time;

    Serial.println(ang_vel_r);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(ENC_R_1, INPUT_PULLUP);
  pinMode(ENC_R_2, INPUT_PULLUP);

  // Attach interrupt for Encoder 1 Channel A
  attachInterrupt(digitalPinToInterrupt(ENC_R_1), read_encoder_right, RISING);
}

void loop() {
  // Set the polling period in milliseconds
  const unsigned long poll_period = 100; // 100 ms

  // Update the angular velocity
  calculate_angular_velocity(poll_period);

  

  
  // Print the angular velocity
  // Serial.print("Angular Velocity (rad/s): ");
  // Serial.println(ang_vel_r);

  // delay(10); // Small delay to avoid flooding the serial monitor
}
