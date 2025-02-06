//ESP32 QT PY Pico version of melontype, simplified to use plain ole led.
//esp32cheesewheel01.ino
//Adapted from Teensy 4.0 version cas07

#include <IBusBM.h>
#include "Adafruit_H3LIS331.h"
#include <SimpleKalmanFilter.h>
#include <Wire.h>
#include "header.h"
#include <esp_now.h>
#include <WiFi.h>


// Create objects
IBusBM ibus;
Adafruit_H3LIS331 accel = Adafruit_H3LIS331();
SimpleKalmanFilter kalmanFilter(20, 20, 0.001);

// Timer handles for ESP32
hw_timer_t* pwmTimer = NULL;
hw_timer_t* ledTimer = NULL;

void setup() {
  delay(1000);                            // little delay so serial message can show up
  Serial.begin(115200, SERIAL_8N1, 256);  //esp32 qt py pico
  //Serial.begin(115200);  // esp32 qt py s3
  Serial.println("esp32cheesewheel07.ino");
  DEBUG_PRINTLN("Initializing ESP32 Meltybrain...");
  initLED();
  initESCs();
  initIBus();
  initAccel();
  initESPNowDebug();
}

void IRAM_ATTR updateLED() {
  if (!ledInterruptsEnabled) return;

  bool redState = (ledPatternR >> ledPosition) & 0x01;
  bool greenState = (ledPatternG >> ledPosition) & 0x01;
  bool blueState = (ledPatternB >> ledPosition) & 0x01;

  digitalWrite(PIN_LED_R, redState);
  digitalWrite(PIN_LED_G, greenState);
  digitalWrite(PIN_LED_B, blueState);

  ledPosition = (ledPosition + 1) & 0x0F;
}

void setLedPattern(uint16_t R, uint16_t G, uint16_t B) {
  ledPatternR = R;
  ledPatternG = G;
  ledPatternB = B;
}

void initLED() {
  DEBUG_PRINTLN("Initializing RGB LEDs");
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  digitalWrite(PIN_LED_R, LOW);
  digitalWrite(PIN_LED_G, LOW);
  digitalWrite(PIN_LED_B, LOW);

  // Set up timer for LED updates (50ms intervals)
  ledTimer = timerBegin(0, 80, true);  // 80MHz / 80 = 1MHz time base
  timerAttachInterrupt(ledTimer, &updateLED, true);
  timerAlarmWrite(ledTimer, 50000, true);  // 50ms intervals
  timerAlarmEnable(ledTimer);
  DEBUG_PRINTLN("RGB LEDs initialized and timer enabled");
}

void initESCs() {
  DEBUG_PRINT("ESCs ");
  ledcSetup(0, ESC_SERVO_RATE, 16);  // Channel 0, 400Hz, 16-bit resolution
  ledcSetup(1, ESC_SERVO_RATE, 16);  // Channel 1, 400Hz, 16-bit resolution
  ledcAttachPin(PIN_ESC1, 0);        // Attach ESC1 to channel 0
  ledcAttachPin(PIN_ESC2, 1);        // Attach ESC2 to channel 1
  DEBUG_PRINTF("on pins %d and %d ", PIN_ESC1, PIN_ESC2);

  // Initialize ESCs with neutral signal (1500μs)
  int neutralPWM = throttleToPWM(0.0);  // This will give us ~4915 for 1500μs
  ledcWrite(0, neutralPWM);
  ledcWrite(1, neutralPWM);
  DEBUG_PRINTF("initialized with neutral pulse %d\n", neutralPWM);
}

void initIBus() {
  DEBUG_PRINT("Starting Ibus ");
  Serial2.begin(115200, SERIAL_8N1, PIN_IBUS, -1);  // RX only
  ibus.begin(Serial2, IBUSBM_NOTIMER);
  DEBUG_PRINTF("on RX pin %d ", PIN_IBUS);
  DEBUG_PRINT(" waiting for remote signal");

  while (!ibus.readChannel(0)) {
    ibus.loop();
    DEBUG_PRINT_RATE(".");
    setLedPattern(LED_PATTERN_RADIO_ERROR);  //
    delay(100);
  }
  // [Rest of function remains the same]

  DEBUG_PRINT("found. ");
  // Check throttle position
  int throttle = ibus.readChannel(2);
  setLedPattern(LED_PATTERN_THROTTLE_ERROR);
  while (throttle > 1050) {
    DEBUG_PRINTF_RATE("Please lower throttle to zero, currently %d\n", throttle);
    throttle = ibus.readChannel(2);
    ibus.loop();
  }
  setLedPattern(LED_PATTERN_GREEN_F);
  DEBUG_PRINTLN(" Radio initialized");
}

void initAccel() {
  DEBUG_PRINT("Accelerometer ");
  // Initialize I2C for accelerometer
  Wire.begin();  // Uses default SDA/SCL pins
  DEBUG_PRINTF("at I2C %d on pins SDA %d and SCL %d ", I2C_ACCEL, SDA, SCL);
  if (!accel.begin_I2C(I2C_ACCEL)) {  //sparkfun 0x19
    DEBUG_PRINT(" init failed!");
    setLedPattern(LED_PATTERN_ERROR);  // Red flashing error pattern
    while (1) delay(100);
  }
  DEBUG_PRINT("found. Configured with ");
  // Configure accelerometer
  accel.setDataRate(LIS331_DATARATE_400_HZ);  // adjust debug print val if changed
  accel.setRange(H3LIS331_RANGE_400_G);
  DEBUG_PRINTF("rate %dhz range %dg ", 400, 400);
  accelCalibration();
  DEBUG_PRINTLN("Complete.");
}

int throttleToPWM(float throttle) {
  // Convert throttle (-1.0 to 1.0) to PWM value (0 to 65535)
  // pulses at 400Hz (2500us period):
  //1000μs = (1000/2500) * 65535 ≈ 26214 , 1500μs  ≈ 39321 , 2000μs  ≈ 52428
  throttle = constrain(throttle, -1.0, 1.0);
  return map(throttle * 1000, -1000, 1000, 26214, 52428);  // 1000-2000us scaled to 16-bit
}

void setThrottle(float throttle1, float throttle2) {
  // Apply motor direction constants without changing throttle vals
  ledcWrite(0, throttleToPWM(throttle1 * MOTOR1_DIRECTION));
  ledcWrite(1, throttleToPWM(throttle2 * MOTOR2_DIRECTION));
}

void accelCalibration() {
  DEBUG_PRINT("Calibrating ");
  setLedPattern(LED_PATTERN_CALIBRATION);  // Yellow alternating for calibration
  float xSum = 0, ySum = 0, zSum = 0;
  const int sampleCount = 150;
  for (int i = 0; i < sampleCount; i++) {
    sensors_event_t s;
    accel.getEvent(&s);
    xSum += s.acceleration.x;
    ySum += s.acceleration.y;
    zSum += s.acceleration.z;
    delayMicroseconds(2500);
    DEBUG_PRINT_RATE(".");
  }
  accelOffsetX = xSum / sampleCount;
  accelOffsetY = ySum / sampleCount;
  accelOffsetZ = zSum / sampleCount;
  setLedPattern(LED_PATTERN_GREEN_F);  // Green flash for calibration complete
  DEBUG_PRINTF(" found offsets X: %f Y: %f Z:%f .", accelOffsetX, accelOffsetY, accelOffsetZ);
}

void updateInputs() {
  ibus.loop();
  // Get raw stick values and normalize to -1.0 to 1.0
  int horiz = ibus.readChannel(0);
  int vert = ibus.readChannel(1);
  stickVert = (vert - 1500) / 500.0;
  stickHoriz = (horiz - 1500) / 500.0;

  stickAngle = atan2(stickVert, stickHoriz) / (2 * PI);  // -0.5 to 0.5 w/ 0 at x+ axis
  //  stickAngle = atan2(stickHoriz, stickVert) / (2 * PI); //-0.5 to 0.5 w/ 0 at y+ axis
  stickLength = sqrt(sq(stickVert) + sq(stickHoriz));
  stickLength = stickLength > 1.0 ? 1.0 : stickLength;

  // Clear very small stick movements 10% dead band in center
  if (stickLength < 0.05) {
    stickAngle = 0;
    stickLength = 0;
  }

  throttle = (ibus.readChannel(2) - 1000) / 1000.0;
  rudderInput = ibus.readChannel(3);
  leftKnob = ibus.readChannel(4);
  toggleLeft3way = ibus.readChannel(6);  //left hand 3-way

  // Handle 3-way switch for knob function
  if (toggleLeft3way > 1500) {
    kalmanInput = leftKnob;
  } else if (toggleLeft3way <= 1500) {
    radiusInput = leftKnob;
  }

  rudderInput = map(rudderInput, 1000, 2000, -100, 100);  // -100 to 100, used as fine trim to radiussize

  float normalizedRadius = (constrain(radiusInput + rudderInput, 1000.0f, 2000.0f) - 1000.0f) / 1000.0f;
  radiusSize = constrain(
    MIN_RADIUS + (MAX_RADIUS - MIN_RADIUS) * normalizedRadius,
    MIN_RADIUS,
    MAX_RADIUS);

  // Calculate Kalman filter process noise
  kalmanQ = constrain(kalmanInput, 1000.0f, 2000.0f);
  kalmanQ = 0.5f + (1.0f / pow(10, map(kalmanQ, 1000, 2000, 0, 3)));  // better at 1.5 than at .5
  kalmanFilter.setProcessNoise(kalmanQ);

  // Get heading offset and right 3-way switch
  headingOffset = (ibus.readChannel(5) - 1500) / 1000.0;   // -0.5 to 0.5
  toggleRight3way = (ibus.readChannel(7) - 1500) / 500.0;  // normalized -1.0 to 1.0
}

bool rc_signal_is_healthy() {
  uint32_t now = millis();
  bool res = false;

  if (now - last_ibus_seen_millis < 500) {
    res = true;
  } else {
    ibus.loop();
  }

  if (last_cnt_rec != ibus.cnt_rec || last_cnt_poll != ibus.cnt_poll) {
    res = true;
    last_cnt_rec = ibus.cnt_rec;
    last_cnt_poll = ibus.cnt_poll;
    last_ibus_seen_millis = now;
  }

  return res;
}

float getSpinAcceleration() {
  sensors_event_t s;
  accel.getEvent(&s);

  // 1. Validate raw Z-axis acceleration
  if (isnan(s.acceleration.z) || isinf(s.acceleration.z)) {
    DEBUG_PRINTF(" Bad Z-accel: %.2f (X=%.2f, Y=%.2f)\n", s.acceleration.z, s.acceleration.x, s.acceleration.y);
    return estimated_accel;  // Return last valid estimate
  }

  // 2. Apply calibration offsets
  float z = s.acceleration.z - accelOffsetZ;

  // 3. Validate calibrated Z-axis acceleration
  if (isnan(z) || isinf(z)) {
    DEBUG_PRINTF(" Bad calibrated Z: %.2f\n", z);
    return estimated_accel;  // Skip Kalman update
  }

  // 4. Update Kalman filter only with valid data
  estimated_accel = kalmanFilter.updateEstimate(z);
  return estimated_accel;
}

float calculateRPS() {
  float spinAccel = getSpinAcceleration();
  // 1. Check accelerometer data
  if ((spinAccel <= 0) || isnan(spinAccel) || isinf(spinAccel)) {
    DEBUG_PRINTF(" Invalid accel data: %1.2f\n", spinAccel);
    return RPS_THRESHOLD;  // Fallback to safe value
  }

  // 2. Check radius size
  if (radiusSize <= 0 || isnan(radiusSize) || isinf(radiusSize)) {
    DEBUG_PRINTF(" Invalid radius: %.4f\n", radiusSize);
    return RPS_THRESHOLD;  // Fallback to safe value
  }

  // 3. Calculate omega (angular velocity)
  float omega = sqrt(spinAccel / radiusSize);
  if (isnan(omega) || isinf(omega)) {
    DEBUG_PRINTF(" Invalid omega: %.2f (spinAccel: %.2f, radius: %.4f)\n", omega, spinAccel, radiusSize);
    return RPS_THRESHOLD;  // Fallback to safe value
  }

  // 5. Convert omega to RPS
  float rps = omega / (2 * PI);
  if (isnan(rps) || isinf(rps)) {
    DEBUG_PRINTF(" Invalid RPS: %.2f (omega: %.2f)\n", rps, omega);
    return RPS_THRESHOLD;  // Fallback to safe value
  }

  // 6. Update maxRPS if valid
  if (rps > maxRPS && !isnan(rps) && !isinf(rps)) {
    maxRPS = rps;
  }

  // 7. Ensure RPS does not fall below threshold
  if (rps < RPS_THRESHOLD) {
    rps = RPS_THRESHOLD;
  }

  return rps;
}

void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  // Optional: Add send status handling
}

void initESPNowDebug() {
  DEBUG_PRINT("ESP-NOW ");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();  // We don't want to connect to any WiFi network
  WiFi.channel(1);    // Use this instead of esp_wifi_set_channel

  if (esp_now_init() != ESP_OK) {  // Init ESP-NOW
    DEBUG_PRINTLN("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 1;  // Must match the channel set above
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    DEBUG_PRINTLN("Failed to add peer");
    return;
  }

  DEBUG_PRINTLN(" initialized");
}

void sendDebugData() {
  esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t*)&debugBuffer, sizeof(debugBuffer));
}

void TankDrive() {
  float x = stickHoriz / 6.0;
  float y = stickVert / 5.0;
  float m1 = y + x;
  float m2 = y - x;
  m1 = modulateThrottle(m1, 0);
  m2 = modulateThrottle(m2, 1);
  setThrottle(m1, m2);
}

float modulateThrottle(float inputThrottle, int idx) {
  unsigned long now = millis();
  if (now > endBoost[idx] + 100) {
    endBoost[idx] = 0;
    return inputThrottle;
  }
  if (now > endBoost[idx] || inputThrottle > boostThreshold) {
    isBoost[idx] = false;
    return inputThrottle;
  }
  if (endBoost[idx] == 0) {
    isBoost[idx] = true;
    endBoost[idx] = now + baseCycleTime;
  }
  if (!isBoost[idx]) return inputThrottle;
  return boostSpeed;
}

void meltybrainDrive() {
  float rps = calculateRPS();
  unsigned long revTimeMicros = 1000000 / rps;
  unsigned long usRevStartTime = micros();  // Start time of the revolution
  ledInterruptsEnabled = false;             // Bypass LED interrupt action

  // Declare variables outside the loop so they can go to debug
  float ph1 = 0.0;                                                     // Throttle phase for motor 1
  float ledPhase = 0.0;                                                // LED phase based on headingOffset
  float widthScale = stickLength > throttle ? stickLength : throttle;  // Width scale for LED timing

  while (true) {                                                     // HOT loop for one revolution
    unsigned long currentTimeMicros = micros() - usRevStartTime;     // Microseconds into the loop
    if (throttle < ZERO_THRESHOLD || currentTimeMicros > 2000000) {  // 2sec timeout emergency exit
      DEBUG_PRINTLN(" Throttle zero or Timeout");
      break;
    }
    if (currentTimeMicros >= revTimeMicros) break;  // Rotation complete

    // LED phase calculation based on headingOffset
    ledPhase = (currentTimeMicros + (headingOffset * revTimeMicros)) / revTimeMicros * M_PI * 2.0f;
    float cos_ledPhase = cosf(ledPhase);

    // Update RGB LED based on ledPhase
    bool ledOn = cos_ledPhase > 0.7071 * (1.4 - widthScale * 0.9);
    if (ledOn) {
      digitalWrite(PIN_LED_R, HIGH);
      digitalWrite(PIN_LED_G, HIGH);  // White flash for heading indicator
      digitalWrite(PIN_LED_B, HIGH);
    } else {
      digitalWrite(PIN_LED_R, LOW);
      digitalWrite(PIN_LED_G, LOW);
      digitalWrite(PIN_LED_B, LOW);
    }

    // Throttle phase calculation based on stickAngle
    float timeToForward = stickAngle * revTimeMicros;
    float timeToBackward = timeToForward + 0.5 * revTimeMicros;
    if (timeToBackward > revTimeMicros) {
      timeToBackward -= revTimeMicros;
    }

    ph1 = ((currentTimeMicros + timeToForward) / revTimeMicros) * M_PI * 2.0f;
    float ph2 = ((currentTimeMicros + timeToBackward) / revTimeMicros) * M_PI * 2.0f;
    float cos_ph1 = cosf(ph1);
    float cos_ph2 = cosf(ph2);

    float th1 = ((cos_ph1 * 0.25f * stickLength) + throttle) > 0 ? ((cos_ph1 * 0.25f * stickLength) + throttle) : 0;
    float th2 = ((cos_ph2 * 0.25f * stickLength) + throttle) > 0 ? ((cos_ph2 * 0.25f * stickLength) + throttle) : 0;

    setThrottle(th1, -th2);  // -th2 because spinning CW
  }

  ledInterruptsEnabled = true;  // Resume LED interrupt action

  // Increment rotation counter
  rotationCounter++;

  // Send debug data every 5 rotations when telemode on
  if (telemode && (rotationCounter % 5 == 0)) {
    // Update debug buffer after the loop
    debugBuffer.usRevStartTime = usRevStartTime;
    debugBuffer.rps = rps;
    debugBuffer.stickAngle = stickAngle;
    debugBuffer.stickLength = stickLength;
    debugBuffer.headingOffset = headingOffset;
    debugBuffer.throttle = throttle;
    debugBuffer.ph1 = ph1 * 0.159155;            // Add throttle phase for motor 1 normalized -1 to 1
    debugBuffer.ledPhase = ledPhase * .159155;  // Add LED phase

    // Send debug data
    sendDebugData();
  }
}

void loop() {
  unsigned long currentLoopStart = micros();
  ibus.loop();

  if (!rc_signal_is_healthy()) {
    DEBUG_PRINT_RATE("Lost radio signal \n");
    setLedPattern(LED_PATTERN_RADIO_ERROR);  // Red flashing error pattern
    setThrottle(0, 0);
    delay(1);
    lastLoopTime = micros() - currentLoopStart;
    return;
  }

  updateInputs();

  bool meltyMode = (throttle > ZERO_THRESHOLD);
  bool tankMode = (throttle < ZERO_THRESHOLD);
  telemode = (toggleRight3way < 0);  // Use toggleRight3way to enable/disable telemode

  if (tankMode) {                        // Tank mode
    setLedPattern(LED_PATTERN_GREEN_S);  // Solid green
    TankDrive();
  } else {                           // Melty mode
    setLedPattern(LED_PATTERN_OFF);  // Turn off LEDs
    meltybrainDrive();
  }
}