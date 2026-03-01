#include <Joystick.h>

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK,
  17, 0, true, true, false, false, false, false, false, false, false, false, false);

// --- CALIBRATION ---
const int xMin = 724; const int xMax = 832;
const int yMin = 32;  const int yMax = 132;
const int deadzone = 4;

// --- SMOOTHING VARIABLES ---
float smoothedX = 512;
float smoothedY = 512;
// Tuning: 0.05 is very smooth but slightly laggy, 0.5 is twitchy. 
// Try 0.2 first.
const float filterWeight = 0.2; 

// --- 74HC165 Pins ---
const int loadPin  = 8;   
const int clockPin = 9;   
const int dataPin  = 10;  
const int numButtons = 17; 

void setup() {
  pinMode(loadPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, INPUT);
  Joystick.setXAxisRange(0, 1023);
  Joystick.setYAxisRange(0, 1023);
  Joystick.begin();
}

void loop() {
  // 1. Read and Constrain
  int rawX = analogRead(A0);
  int rawY = analogRead(A1);

  // 2. Map to target range
  float targetX = map(constrain(rawX, xMin, xMax), xMin, xMax, 0, 1023);
  float targetY = map(constrain(rawY, yMin, yMax), yMin, yMax, 0, 1023);

  // 3. LOW-PASS FILTER (The Smoothing Part)
  // formula: NewValue = (Current * weight) + (Previous * (1 - weight))
  smoothedX = (targetX * filterWeight) + (smoothedX * (1.0 - filterWeight));
  smoothedY = (targetY * filterWeight) + (smoothedY * (1.0 - filterWeight));

  // 4. Apply Deadzone to the smoothed value
  int finalX = (int)smoothedX;
  int finalY = (int)smoothedY;

  if (abs(finalX - 512) < (deadzone * 10)) finalX = 512;
  if (abs(finalY - 512) < (deadzone * 10)) finalY = 512;

  Joystick.setXAxis(finalX);
  Joystick.setYAxis(finalY);

  // 5. BUTTONS (74HC165)
  digitalWrite(loadPin, LOW);
  delayMicroseconds(5);
  digitalWrite(loadPin, HIGH);

  for (int i = 0; i < numButtons; i++) {
    int bitVal = digitalRead(dataPin);
    Joystick.setButton(i, bitVal);
    digitalWrite(clockPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(clockPin, LOW);
  }

  delay(5); // Increased polling rate for better filter response
}
