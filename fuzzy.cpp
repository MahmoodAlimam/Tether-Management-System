#include"fuzzy.h"


/**
 * @brief Calculates the membership degree for a triangular fuzzy set.
 * @param x The input value.
 * @param a The left base point of the triangle.
 * @param b The peak point of the triangle.
 * @param c The right base point of the triangle.
 * @return The membership degree (0.0 to 1.0).
 */
float getMembership(float x, float a, float b, float c) {
  if (x <= a || x >= c) {
    return 0.0;
  } else if (x >= a && x <= b) {
    return (x - a) / (b - a);
  } else { // x >= b && x <= c
    return (c - x) / (c - b);
  }
}

/**
 * @brief Maps a float value from one range to another.
 * This is similar to Arduino's map() function but for floats.
 * @param x The input value to map.
 * @param in_min The minimum value of the input's current range.
 * @param in_max The maximum value of the input's current range.
 * @param out_min The minimum value of the output's desired range.
 * @param out_max The maximum value of the output's desired range.
 * @return The mapped float value.
 */
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief Sets the motor speed by sending a specific pulse width to the ESC.
 * @param pulseWidth The desired pulse width in microseconds (e.g., 1000-2000).
 * Clamped between MIN_PULSE and MAX_PULSE.
 */
void setMotorPulse(Servo esc,int pulseWidth) {
  // Ensure the pulse width is within the defined safe range
  if (pulseWidth < MIN_PULSE) pulseWidth = MIN_PULSE;
  if (pulseWidth > MAX_PULSE) pulseWidth = MAX_PULSE;

  esc.writeMicroseconds(pulseWidth); // Send the pulse to the ESC
}

/**
 * @brief Stops the motor by sending the neutral pulse to the ESC.
 */
void stopMotor(Servo esc) {
  setMotorPulse(esc, STOP_PULSE);
}
