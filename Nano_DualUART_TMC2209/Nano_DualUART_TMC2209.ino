/**
 * TMC2209 Dual Motor Controller with SoftwareSerial UART Switching
 * 
 * Based on original concept by: Teemu Mäntykallio
 * Rewritten by: Kletternaut
 * 
 * Enables dual TMC2209 control on Arduino Nano!
 * - Maintain full microstep flexibility (1-256 steps) on both drivers
 * - Provide separate UART channels without address conflicts
 * 
 * Hardware Requirements:
 * - Arduino Nano V3 (tested)
 * - 2x TMC2209 (BIGTREETECH V1.3) stepper drivers (tested)
 * 
 * Features:
 * - Dual TMC2209 stepper drivers with separate SoftwareSerial connections
 * - UART switching system for reliable communication
 * - Stallguard-based sensorless homing
 * - Real-time motor diagnostics and control
 * - Memory-optimized for Arduino Nano constraints
 * 
 * Version: 2.0 - Complete rewrite
 * Date: September 2025
 */

#include <TMCStepper.h>
#include <SoftwareSerial.h>

// First TMC2209 Driver
#define EN_PIN            2   // Enable
#define DIR_PIN           3   // Direction
#define STEP_PIN          4   // Step
#define SW_RX             7   // SoftwareSerial RX (connect to TX of TMC2209)
#define SW_TX             6   // SoftwareSerial TX (connect to RX of TMC2209)
#define DIAG_PIN          8   // DIAG1 from TMC2209 connect to this pin
#define DRIVER_ADDRESS 0b00   // TMC2209 Driver address

// Second TMC2209 Driver
#define EN2_PIN           A0  // Enable
#define DIR2_PIN          12  // Direction
#define STEP2_PIN         11  // Step
#define SW2_RX            9   // SoftwareSerial RX (connect to TX of TMC2209)
#define SW2_TX            10  // SoftwareSerial TX (connect to RX of TMC2209)
#define DIAG2_PIN         5   // DIAG1 from TMC2209 connect to this pin (D5 with interrupt support)
#define DRIVER2_ADDRESS 0b00 // Both can use 0b00 (separate UART)

// SoftwareSerial instances for the two TMC2209 drivers
SoftwareSerial serialPort1(SW_RX, SW_TX);    // First driver
SoftwareSerial serialPort2(SW2_RX, SW2_TX);  // Second driver

// Global flag for active SoftwareSerial
bool useDriverA = true;

// Option to disable homing for testing
bool enableHoming = true;  // Homing enabled since UART works

//Stallguard values for each driver(0-255), lower number -> higher sensitivity.
#define STALLA_VALUE 5    // Conservative setting for reliable homing (range: 0-255, typical: 5-20)
#define STALLB_VALUE 10   // Slightly more sensitive due to different load characteristics (range: 0-255)

// Motor currents in mA
#define MOTOR_A_CURRENT 700  // Adjust based on motor specifications and thermal limits (range: 50-2000mA)
#define MOTOR_B_CURRENT 700  // Match to Motor A for consistent torque output (range: 50-2000mA)

#define RA_SENSE 0.11f // Sense resistor value - must match physical resistor on driver board (typical: 0.11Ω)
#define RB_SENSE 0.11f // Standard value for BIGTREETECH TMC2209 v1.3 (range: 0.05-0.25Ω)


TMC2209Stepper driverA(&serialPort1, RA_SENSE, DRIVER_ADDRESS);
TMC2209Stepper driverB(&serialPort2, RB_SENSE, DRIVER2_ADDRESS);

constexpr uint32_t steps_per_round = 200*(60/16); // 200 steps/rev * gear ratio (60:16) = 750 steps per output revolution

#include <AccelStepper.h>
AccelStepper stepperA = AccelStepper(stepperA.DRIVER, STEP_PIN, DIR_PIN);
AccelStepper stepperB = AccelStepper(stepperB.DRIVER, STEP2_PIN, DIR2_PIN);

bool startup = true; // Prevents multiple homing cycles on reset
bool stalled_A = false; // Interrupt flag for Motor A stallguard detection (set by ISR)
bool stalled_B = false; // Interrupt flag for Motor B stallguard detection (set by ISR)

void stallInterruptX(){ // ISR triggered when Motor A stalls (DIAG pin goes HIGH)
stalled_A = true;
}

void stallInterruptB(){ // ISR triggered when Motor B stalls (DIAG pin goes HIGH)
stalled_B = true;
}

// Function to switch between UART connections
void switchToDriverB() {
  if(useDriverA) {
    serialPort1.end();
    delay(50);
    serialPort2.begin(115200);
    delay(100);
    useDriverA = false;
  }
}

void switchToDriverA() {
  if(!useDriverA) {
    serialPort2.end();
    delay(50);
    serialPort1.begin(115200);
    delay(100);
    useDriverA = true;
  }
}

// Diagnostic function for driver settings (compact)
void diagnoseTreiber() {
  Serial.println(F("=== DIAGNOSIS ==="));
  
  // Check Driver A
  switchToDriverA();
  delay(100);
  Serial.println(F("Driver A:"));
  Serial.print(F("  Current: "));
  Serial.println(driverA.rms_current());
  Serial.print(F("  Microsteps: "));
  Serial.println(driverA.microsteps());
  
  // Check Driver B
  switchToDriverB();
  delay(100);
  Serial.println(F("Driver B:"));
  Serial.print(F("  Current: "));
  Serial.println(driverB.rms_current());
  Serial.print(F("  Microsteps: "));
  Serial.println(driverB.microsteps());
  
  switchToDriverA(); // Back to Driver A
  Serial.println(F("================"));
}

void motorBHome()
{
  Serial.println(F("Motor B: Homing..."));
  
  // Switch to Driver B UART
  switchToDriverB();
  
  // Activate and configure Stallguard
  driverB.TCOOLTHRS(0xFFFFF); 
  driverB.SGTHRS(STALLB_VALUE);
  
  stepperB.move(-100*steps_per_round);
  uint32_t startTime = millis();
  uint32_t lastSgCheck = millis();
  bool validSgValues = false;
  stalled_B = false;  // Reset interrupt flag
  
  while(1)
    {
      stepperB.run();

      // SG_RESULT polling for debug (like Motor A)
      if(millis() - lastSgCheck > 200) {  // Check every 200ms
        uint16_t sg_result = driverB.SG_RESULT();
        Serial.print(F("B SG: "));
        Serial.println(sg_result);
        lastSgCheck = millis();
        
        // Check if UART works (valid SG values)
        if(sg_result > 0 && sg_result < 500) {
          validSgValues = true;
        }
        
        // Stall detection only with valid UART values
        if(validSgValues && sg_result > 0 && sg_result < 15) {  // Moderate threshold
          Serial.println(F("Motor B: Stall detected (Polling)!"));
          stepperB.setCurrentPosition(0);
          break;
        }
      }

      // Interrupt-based stall detection (as backup)
      if(stalled_B) {
        Serial.println(F("Motor B: Stall detected (Interrupt)!"));
        stepperB.setCurrentPosition(0);
        stalled_B = false;
        break; 
      }
      
      // Timeout after 15 seconds
      if(millis() - startTime > 15000) {
        Serial.println(F("Motor B: Timeout"));
        stepperB.setCurrentPosition(0);
        break;
      }
      
      // If no valid SG values after 5 seconds → time-based homing
      if(!validSgValues && millis() - startTime > 5000) {
        Serial.println(F("Motor B: Time-based homing"));
        stepperB.setCurrentPosition(0);
        break;
      }
    }

  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println(F("Motor B: Moving to position..."));
  stepperB.setMaxSpeed(10*steps_per_round);
  stepperB.setAcceleration(50*steps_per_round);
  stepperB.moveTo(1250);

  while(stepperB.distanceToGo())
  stepperB.run();
  
  // Switch back to Driver A UART
  switchToDriverA();
  
  Serial.print(F("Motor B finished - Pos: "));
  Serial.println(stepperB.currentPosition());
}

void motorAHome()
{
  Serial.println(F("Motor A: Homing..."));
  
  // Ensure we are on Driver A UART
  switchToDriverA();
  
  // Activate and configure Stallguard
  driverA.TCOOLTHRS(0xFFFFF); // 20bit max - Stallguard always active
  driverA.SGTHRS(STALLA_VALUE);
  
  stepperA.move(-100*steps_per_round);
  uint32_t startTime = millis();
  uint32_t lastSgCheck = millis();
  bool validSgValues = false;
  
  while(1)
    {
      stepperA.run();

      // SG_RESULT polling for debug
      if(millis() - lastSgCheck > 200) {  // Check every 200ms
        uint16_t sg_result = driverA.SG_RESULT();
        Serial.print(F("A SG: "));
        Serial.println(sg_result);
        lastSgCheck = millis();
        
        // Check if UART works (valid SG values)
        if(sg_result > 0 && sg_result < 500) {
          validSgValues = true;
        }
        
        // Stall detection only with valid UART values
        if(validSgValues && sg_result > 0 && sg_result < 15) {  // Moderate threshold
          Serial.println(F("Motor A: Stall detected!"));
          stepperA.setCurrentPosition(0);
          break;
        }
      }

      // Original interrupt-based detection
      if(stalled_A){
        Serial.println(F("Motor A: Interrupt Stall!"));
        stepperA.setCurrentPosition(0);
        break; 
      }
      
      // Timeout after 15 seconds or if no valid SG values
      if(millis() - startTime > 15000) {
        Serial.println(F("Motor A: Timeout"));
        stepperA.setCurrentPosition(0);
        break;
      }
      
      // If no valid SG values after 5 seconds → time-based homing
      if(!validSgValues && millis() - startTime > 5000) {
        Serial.println(F("Motor A: Time-based homing"));
        stepperA.setCurrentPosition(0);
        break;
      }
    }

    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);

    Serial.println(F("Motor A: Moving to position..."));
    stepperA.setMaxSpeed(10*steps_per_round);
    stepperA.setAcceleration(50*steps_per_round);
    stepperA.moveTo(2000);

    while(stepperA.distanceToGo())
    stepperA.run();
    
    Serial.print(F("Motor A finished - Pos: "));
    Serial.println(stepperA.currentPosition());
}


void setup() {
    
    Serial.begin(115200);        // Serial for debug output
    while(!Serial);
    Serial.println(F("TMC2209 Dual Motor Test"));
    
    // Initialize only serialPort1
    serialPort1.begin(115200);  // First TMC2209 driver
    delay(100);
    
    // Do NOT initialize serialPort2 simultaneously
    // serialPort2.begin(115200);  // Will be activated later when needed

    pinMode(LED_BUILTIN, OUTPUT);
    
    // Define Enable pins as outputs
    pinMode(EN_PIN, OUTPUT);
    pinMode(EN2_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW);   // Activate driver (LOW = enabled)
    digitalWrite(EN2_PIN, LOW);  // Activate driver (LOW = enabled)

    // Use interrupt pins for both motors
    attachInterrupt(digitalPinToInterrupt(DIAG_PIN), stallInterruptX, RISING);
    attachInterrupt(digitalPinToInterrupt(DIAG2_PIN), stallInterruptB, RISING);  // D5 supports interrupts
    
    driverA.begin();             // Initiate pins and registers
    delay(100);
    
    // Configure Driver A (test_connection() is unreliable with SoftwareSerial)
    Serial.println(F("Configuring Driver A..."));
    driverA.rms_current(MOTOR_A_CURRENT);    // Use defined constant
    driverA.pwm_autoscale(1);
    driverA.microsteps(16);
    driverA.TCOOLTHRS(0xFFFFF); // 20bit max - Stallguard always active (range: 0-0xFFFFF)
    driverA.SGTHRS(STALLA_VALUE);
    Serial.print(F("A Current: "));
    Serial.println(MOTOR_A_CURRENT);
    delay(200);  // Longer pause

    // Switch to Driver B UART
    Serial.println(F("Switching to Driver B..."));
    serialPort1.end();  // End Driver A UART
    delay(100);
    serialPort2.begin(115200);  // Start Driver B UART
    delay(200);

    driverB.begin();             // Initiate pins and registers
    delay(100);
    
    // Configure Driver B (test_connection() is unreliable with SoftwareSerial)
    Serial.println(F("Configuring Driver B..."));
    driverB.rms_current(MOTOR_B_CURRENT);    // Use defined constant
    driverB.pwm_autoscale(1);
    driverB.microsteps(16);
    driverB.TCOOLTHRS(0xFFFFF); // 20bit max - Stallguard always active (range: 0-0xFFFFF)
    driverB.SGTHRS(STALLB_VALUE);
    Serial.print(F("B Current: "));
    Serial.println(MOTOR_B_CURRENT);
    delay(100);
    
    // Back to serialPort1 for normal operation
    Serial.println(F("Back to Driver A..."));
    serialPort2.end();  // End Driver B UART
    delay(100);
    serialPort1.begin(115200);  // Restart Driver A UART
    delay(200);

    stepperA.setMaxSpeed(1.25*steps_per_round); // 100mm/s @ 80 steps/mm
    stepperA.setAcceleration(10*steps_per_round); // 2000mm/s^2
    stepperA.setEnablePin(EN_PIN);
    stepperA.setPinsInverted(false, false, true);
    stepperA.enableOutputs();

    stepperB.setMaxSpeed(1.25*steps_per_round); // 100mm/s @ 80 steps/mm
    stepperB.setAcceleration(10*steps_per_round); // 2000mm/s^2
    stepperB.setEnablePin(EN2_PIN);
    stepperB.setPinsInverted(false, false, true);
    stepperB.enableOutputs();

    delay(500);  // Pause before homing

    if(enableHoming) {
        Serial.println(F("Starting Motor A Homing..."));
        motorAHome();
        Serial.println(F("Motor A Homing finished"));
        
        Serial.println(F("Starting Motor B Homing..."));
        motorBHome();
        Serial.println(F("Motor B Homing finished"));
    } else {
        Serial.println(F("Homing disabled"));
        stepperA.setCurrentPosition(0);
        stepperB.setCurrentPosition(0);
    }
    
    // Driver diagnosis after initialization
    diagnoseTreiber();
    
    //stepperB.moveTo(2000);
    
}

void loop() {

  
    if(Serial.available()>0){
      char readVal = Serial.read();
      if (readVal == 'a'){
      int steps = Serial.parseInt();
      Serial.print("Motor A moving to position: ");
      Serial.println(steps);
      stepperA.moveTo(steps);
      
      } else if (readVal == 'A'){
      Serial.println("Motor A Homing...");
      motorAHome();
      }

      if (readVal == 'b'){
      int steps = Serial.parseInt();
      Serial.print("Motor B moving to position: ");
      Serial.println(steps);
      stepperB.moveTo(steps);
      
      } else if (readVal == 'B'){
      Serial.println("Motor B Homing...");
      motorBHome();
      }
      
      // New test command for Motor B
      if (readVal == 't'){
      Serial.println("Motor B Test: 10 revolutions forward...");
      stepperB.move(10*steps_per_round);
      }
      
      // Diagnosis command
      if (readVal == 'd'){
      diagnoseTreiber();
      }
      
      // Increase Motor B current
      if (readVal == '+'){
      switchToDriverB();
      int newCurrent = driverB.rms_current() + 100;
      if(newCurrent <= 1200) {
        driverB.rms_current(newCurrent);
        Serial.print("Motor B current increased to: ");
        Serial.print(newCurrent);
        Serial.println(" mA");
      } else {
        Serial.println("Maximum current reached (1200mA)");
      }
      switchToDriverA();
      }
      
      // Decrease Motor B current
      if (readVal == '-'){
      switchToDriverB();
      int newCurrent = driverB.rms_current() - 100;
      if(newCurrent >= 300) {
        driverB.rms_current(newCurrent);
        Serial.print("Motor B current decreased to: ");
        Serial.print(newCurrent);
        Serial.println(" mA");
      } else {
        Serial.println("Minimum current reached (300mA)");
      }
      switchToDriverA();
      }
    }

  

  stepperA.run();
  stepperB.run();
    
}
