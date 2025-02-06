// Pin definitions for ESP32 QT PY Pico
//#define PIN_LED A1      // future sk6812 LED control pin
#define PIN_LED_R MISO    // Red LED on MISO
#define PIN_LED_G SCK    // Green LED on SCK
#define PIN_LED_B MOSI     // Blue LED on MOSI
#define PIN_ESC1 A2     // Left motor
#define PIN_ESC2 A3     // Right motor
#define PIN_IBUS A0     // IBUS on pin 27 (RX2)
#define I2C_ACCEL 0x19  // sparkfun

// Motor direction constants
#define MOTOR1_DIRECTION -1.0  // Set to -1 to reverse motor 1, 1 for normal
#define MOTOR2_DIRECTION -1.0  // Set to -1 to reverse motor 2, 1 for normal

// LED pattern variables
volatile bool ledInterruptsEnabled = true;
uint32_t lastLedUpdate = 0;
volatile uint16_t ledPatternR = 0;  // Red LED pattern
volatile uint16_t ledPatternG = 0;  // Green LED pattern
volatile uint16_t ledPatternB = 0;  // Blue LED pattern
volatile uint8_t ledPosition = 0;   // Current bit position in pattern

// LED pattern definitions
#define LED_PATTERN_ERROR          0xDEDE, 0x0000, 0x0000  // Red flashing sos
#define LED_PATTERN_RADIO_ERROR    0xF0F0, 0x0000, 0x0000  // Red flashing
#define LED_PATTERN_THROTTLE_ERROR 0x0000, 0xF0F0, 0x0F0F  // blue green flashing
#define LED_PATTERN_CALIBRATION    0x5555, 0x5555, 0x0000  // Yellow (R+G) fast
#define LED_PATTERN_GREEN_S        0x0000, 0xFFFF, 0x0000  // solid green
#define LED_PATTERN_GREEN_F        0x0000, 0x0F0F, 0x0000  // Flash green
#define LED_PATTERN_OFF            0x0000, 0x0000, 0x0000  // off // also need to disable led interrupt in meltymode
#define LED_PATTERN_CYCLE          0xc000, 0x0c00, 0x00c0  // cycles rgb

// Constants for ESC control
const int ESC_SERVO_RATE = 400;  // 400Hz
const float ZERO_THRESHOLD = 0.1;

// Variables to store inputs
volatile float stickVert = 0.0;
volatile float stickHoriz = 0.0;
volatile float stickAngle = 0.0;
volatile float stickLength = 0.0;
volatile float throttle = 0.0;
volatile float toggleRight3way = 0.0;
volatile float rudderInput = 0.0;
volatile float radiusInput = 0.0;
volatile float radiusSize = 0.01;
#define MIN_RADIUS 0.001f  // Absolute minimum physical radius 1mm
#define MAX_RADIUS 0.050f  //  max 50mm
volatile float headingOffset = 0.0;
volatile float kalmanQ = 0.01;
volatile float kalmanInput = 0.0;
volatile float leftKnob = 0.0;
volatile float toggleLeft3way = 0.0;

// Acceleration offsets
float accelOffsetX, accelOffsetY, accelOffsetZ;

// Boost control
const float boostThreshold = 0.2;        // Minimum throttle value for motor to start spinning
const float boostSpeed = 0.5;            // Throttle output during "on" period when modulating
const unsigned long baseCycleTime = 50;  // Base cycle time in milliseconds
unsigned long endBoost[2] = { 0, 0 };
bool isBoost[2] = { true, true };  // Flag to track whether we're sending modSpeed or 0

//Sensor measurements
const float RPS_THRESHOLD = 7.0;
volatile float estimated_accel;
volatile float maxRPS = 0.0;  // Stores the maximum RPS value

// IBUS health check variables
volatile uint32_t last_ibus_seen_millis;
volatile uint8_t last_cnt_rec;
volatile uint8_t last_cnt_poll;

// Loop timing variables
//volatile unsigned long loopStartTime = 0;
 unsigned long lastLoopTime = 0;
 unsigned long maxLoopTime = 0;
volatile unsigned long avgLoopTime = 0;
volatile unsigned long loopCount = 0;

  bool telemode =0;  // Use toggleRight3way to enable/disable telemode
// espNOW Receiver MAC Address (replace with your receiver's MAC)
uint8_t receiverMacAddress[] = { 0x3C, 0x61, 0x05, 0x0C, 0x9C, 0x28};  //  ttgo oled board

typedef struct { // espNOW telemetry packet
    uint32_t usRevStartTime;
    float rps;
    float stickAngle;
    float stickLength;
    float headingOffset;
    float throttle;
    float ph1;       // Throttle phase for motor 1
    float ledPhase;  // LED phase based on headingOffset
} debug_data_t;

debug_data_t debugBuffer;

volatile uint16_t rotationCounter;

// Debug Control
#define DEBUG_PRINT_INTERVAL 250     // Print interval in milliseconds
static uint32_t lastDebugPrint = 0;  // Tracks last debug print time

// Regular debug prints - no rate limiting
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINTF(format, ...) Serial.printf(format, __VA_ARGS__)

// Rate-limited debug prints
#define DEBUG_PRINT_RATE(x) \
  do { \
    if ((millis() - lastDebugPrint) > DEBUG_PRINT_INTERVAL) { \
      Serial.print(x); \
      lastDebugPrint = millis(); \
    } \
  } while (0)

#define DEBUG_PRINTLN_RATE(x) \
  do { \
    if ((millis() - lastDebugPrint) > DEBUG_PRINT_INTERVAL) { \
      Serial.println(x); \
      lastDebugPrint = millis(); \
    } \
  } while (0)

#define DEBUG_PRINTF_RATE(format, ...) \
  do { \
    if ((millis() - lastDebugPrint) > DEBUG_PRINT_INTERVAL) { \
      Serial.printf(format, __VA_ARGS__); \
      lastDebugPrint = millis(); \
    } \
  } while (0)