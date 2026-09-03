/*
 * Flow Sensor Diagnostic Test - Enhanced Debug
 * Tests the physical pin and shows detailed calculation info
 */

#include <Arduino.h>

#define FLOW_SENSOR_PIN 16

volatile unsigned long FlowPulseCount = 0;
volatile unsigned long LastPulseTime = 0;

void IRAM_ATTR FlowSensorISR_RISING() {
    FlowPulseCount++;
    LastPulseTime = millis();
}

// Calculate flow rate with debug output
float CalculateFlowRate_Debug(unsigned long& lastMeasureTime, unsigned long& lastPulseCount) {
    unsigned long currentTime = millis();
    unsigned long timeDelta = currentTime - lastMeasureTime;
    unsigned long pulseDelta = FlowPulseCount - lastPulseCount;
    
    Serial.print("Time Delta: ");
    Serial.print(timeDelta);
    Serial.print("ms | Pulse Delta: ");
    Serial.print(pulseDelta);
    Serial.print(" | ");
    
    if (timeDelta == 0) {
        Serial.println("NO TIME DELTA");
        return 0.0;
    }
    
    // Calibration factor (pulses per mL)
    const float CALIB = 4.5;
    float volumeML = (float)pulseDelta / CALIB;
    float timeSeconds = timeDelta / 1000.0;
    float flowRate = volumeML / timeSeconds;
    
    Serial.print("Volume: ");
    Serial.print(volumeML, 2);
    Serial.print("mL | Time: ");
    Serial.print(timeSeconds, 2);
    Serial.print("s | Flow: ");
    Serial.print(flowRate, 2);
    Serial.println(" mL/s");
    
    lastMeasureTime = currentTime;
    lastPulseCount = FlowPulseCount;
    
    return flowRate;
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n=== FLOW SENSOR DEBUG - ENHANCED ===");
    Serial.println("Pin: " + String(FLOW_SENSOR_PIN));
    Serial.println("Calibration: 4.5 pulses/mL");
    Serial.println("Checking calculations every 500ms\n");
    
    pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
    
    // Attach interrupt
    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), FlowSensorISR_RISING, RISING);
    Serial.println("Sensor initialized. Water flowing? Start flowing now!\n");
}

void loop() {
    static unsigned long lastMeasureTime = 0;
    static unsigned long lastPulseCount = 0;
    static unsigned long lastPrint = 0;
    
    if (millis() - lastPrint > 500) {
        Serial.print("Total Pulses: ");
        Serial.print(FlowPulseCount);
        Serial.print(" | ");
        
        float flowRate = CalculateFlowRate_Debug(lastMeasureTime, lastPulseCount);
        
        if (flowRate > 0.1) {
            Serial.println("  ✓ FLOW DETECTED");
        } else {
            Serial.println("  ✗ NO FLOW (< 0.1 mL/s)");
        }
        
        lastPrint = millis();
    }
}
