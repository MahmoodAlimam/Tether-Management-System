#include <Servo.h>         // Include the Servo library for ESC control
#include <Arduino_LSM6DS3.h> // Include the library for the onboard IMU

// --- ESC Control Definitions ---
const int escSignalPin = 5;  // Digital pin 9 for ESC control
//Servo esc;                   // Create a Servo object to control the ESC

// Constants for ESC control (pulse widths in microseconds)
const int STOP_PULSE = 1500; // Neutral/Stop pulse width for the ESC
const int MIN_PULSE = 1100;  // Minimum pulse width (e.g., full reverse)
const int MAX_PULSE = 1900;  // Maximum pulse width (e.g., full forward)
// Adjust MIN_PULSE and MAX_PULSE based on your specific ESC's calibration.

// --- Fuzzy Logic Parameters ---
// Input (Accelerometer Y-axis 'g' values) ranges for membership functions
// These values define the "breakpoints" for the fuzzy sets.
// Adjust these based on how sensitive you want the control to be to tilt.
const float ACCEL_NL_PEAK = -0.7; // Peak for Negative_Large
const float ACCEL_NS_PEAK = -0.3; // Peak for Negative_Small
const float ACCEL_ZERO_PEAK = 0.0; // Peak for Zero
const float ACCEL_PS_PEAK = 0.3;  // Peak for Positive_Small
const float ACCEL_PL_PEAK = 0.7;  // Peak for Positive_Large

// Overlap points for triangular membership functions
// Example: Negative_Large goes from -1.0 to -0.3, peaking at -0.7
// Negative_Small goes from -0.7 to 0.0, peaking at -0.3
const float ACCEL_RANGE_MIN = -1.0; // Absolute min acceleration to consider
const float ACCEL_RANGE_MAX = 1.0;  // Absolute max acceleration to consider

// Output (Motor Command - normalized from -1.0 to 1.0)
// These represent the crisp output values for each fuzzy output set.
const float MOTOR_FR = -1.0; // Full Reverse
const float MOTOR_SR = -0.5; // Slow Reverse
const float MOTOR_ST = 0.0;  // Stop
const float MOTOR_SF = 0.5;  // Slow Forward
const float MOTOR_FF = 1.0;  // Full Forward

// Variables to store accelerometer data
float xAcc, yAcc, zAcc;