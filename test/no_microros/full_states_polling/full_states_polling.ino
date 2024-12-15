// Encoder pins
#define ENC_R_1 0 // Encoder 1, Channel A (YELLOW)
#define ENC_R_2 1 // Encoder 1, Channel B (WHITE)
#define ENC_L_1 2
#define ENC_L_2 3

const int PPR = 990; // Pulses per revolution

// Global variable to store the angular velocity
volatile float ang_vel_r = 0.0; // Angular velocity in rad/s
volatile float ang_vel_l = 0.0; // Angular velocity in rad/s

// Position variable for the encoder
volatile int posi_r = 0;
volatile int posi_l = 0;

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


// Interrupt Service Routine to update encoder position
void read_encoder_left() {
  // Read the state of encoder 1 channel B
  int b = digitalRead(ENC_L_2);

  // Increment or decrement the position based on the state of channel B
  if (b > 0) {
    posi_l++;
  } else {
    posi_l--;
  }
}


void calculate_angular_velocity(unsigned long poll_period_ms) {
  static unsigned long last_poll_time = 0;
  static int last_posi_r = 0;
  static int last_posi_l = 0;

  unsigned long current_time = millis();

  // Check if polling period has elapsed
  if (current_time - last_poll_time >= poll_period_ms) {
    // Get the latest positions
    int current_posi_r = posi_r;
    int current_posi_l = posi_l;

    // Calculate the change in position
    int delta_position_r = current_posi_r - last_posi_r;
    int delta_position_l = current_posi_l - last_posi_l;

    // Update last positions
    last_posi_r = current_posi_r;
    last_posi_l = current_posi_l;

    // Calculate angular velocities (rad/s)
    ang_vel_r = (delta_position_r / (float)PPR) * (2 * PI) / (poll_period_ms / 1000.0);
    ang_vel_l = (delta_position_l / (float)PPR) * (2 * PI) / (poll_period_ms / 1000.0);

    // Update the last poll time
    last_poll_time = current_time;

    // Print angular velocities
    Serial.print("Angular Velocity Right (rad/s): ");
    Serial.println(ang_vel_r);
    Serial.print("Angular Velocity Left (rad/s): ");
    Serial.println(ang_vel_l);
  }
}


void setup() {
  Serial.begin(115200);

  pinMode(ENC_R_1, INPUT_PULLUP);
  pinMode(ENC_R_2, INPUT_PULLUP);
  pinMode(ENC_L_1, INPUT_PULLUP);
  pinMode(ENC_L_2, INPUT_PULLUP);
  // Attach interrupt for Encoder 1 Channel A
  attachInterrupt(digitalPinToInterrupt(ENC_R_1), read_encoder_right, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_L_1), read_encoder_left, RISING);
}

void loop() {
  // Set the polling period in milliseconds
  const unsigned long poll_period = 50; // 50 ms

  // Update the angular velocity
  calculate_angular_velocity(poll_period);


  // Print the angular velocity
  // Serial.print("Angular Velocity (rad/s): ");
  // Serial.println(ang_vel_r);

  // delay(10); // Small delay to avoid flooding the serial monitor
}
