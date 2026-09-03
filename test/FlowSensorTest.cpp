/*
 * Flow Sensor Test
 * This file tests the YF201 flow sensor functionality
 * Measures pulse count and calculates flow rate in mL/sec
 */

#include <Arduino.h>

//======================================================================================
// SENSOR PINS
//======================================================================================
#define FLOW_SENSOR_PIN 16

//======================================================================================
// FLOW SENSOR VARIABLES
//======================================================================================
volatile unsigned long FlowPulseCount = 0;  // Interrupt-safe counter
float CurrentFlowRate = 0.0;                // mL/sec
unsigned long LastFlowMeasureTime = 0;      // Last time flow was measured
unsigned long LastFlowPulseCount = 0;       // Pulse count at last measurement
const float FLOW_SENSOR_CALIBRATION = 4.5;  // YF201 pulses per mL (adjust based on calibration)

//======================================================================================
// INTERRUPT SERVICE ROUTINE
//======================================================================================

// YF201 Flow Sensor ISR - counts pulses from turbine
void IRAM_ATTR FlowSensorISR() {
    FlowPulseCount++;
}

//======================================================================================
// FLOW RATE CALCULATION
//======================================================================================

// Calculate flow rate from pulse count
// Returns flow rate in mL/sec
float CalculateFlowRate() {
    unsigned long currentTime = millis();
    unsigned long timeDelta = currentTime - LastFlowMeasureTime;
    unsigned long pulseDelta = FlowPulseCount - LastFlowPulseCount;
    
    if (timeDelta == 0) return 0.0;
    
    // Convert pulses to mL using calibration factor
    // Flow (mL/sec) = (pulses / calibration_factor) / (time_in_ms / 1000)
    float volumeML = (float)pulseDelta / FLOW_SENSOR_CALIBRATION;
    float flowRate = volumeML / (timeDelta / 1000.0);
    
    LastFlowMeasureTime = currentTime;
    LastFlowPulseCount = FlowPulseCount;
    
    return flowRate;
}

//======================================================================================
// SETUP
//======================================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n=== FLOW SENSOR TEST ===");
    Serial.println("YF201 Flow Sensor Testing");
    Serial.println("Calibration Factor: " + String(FLOW_SENSOR_CALIBRATION) + " pulses/mL");
    Serial.println("Flow Sensor Pin: " + String(FLOW_SENSOR_PIN));
    
    // Setup flow sensor pin and interrupt
    pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), FlowSensorISR, RISING);
    
    FlowPulseCount = 0;
    LastFlowPulseCount = 0;
    LastFlowMeasureTime = millis();
    
    Serial.println("Flow sensor initialized. Water flowing? [Y/N]");
    Serial.println("========================\n");
}

//======================================================================================
// LOOP
//======================================================================================

void loop() {
    // Measure flow rate periodically (every 500ms)
    static unsigned long lastFlowCheck = 0;
    
    if (millis() - lastFlowCheck > 500) {
        CurrentFlowRate = CalculateFlowRate();
        lastFlowCheck = millis();
        
        // Print detailed flow information
        Serial.print("Time: ");
        Serial.print(millis() / 1000.0);
        Serial.print("s | Pulses: ");
        Serial.print(FlowPulseCount);
        Serial.print(" | Flow Rate: ");
        Serial.print(CurrentFlowRate, 2);
        Serial.println(" mL/sec");
        
        // Detect flow presence
        if (CurrentFlowRate > 0.1) {
            Serial.println("  [FLOW DETECTED]");
        } else {
            Serial.println("  [NO FLOW]");
        }
    }
}
