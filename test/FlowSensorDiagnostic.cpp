/*
 * Flow Sensor Diagnostic Test
 * Tests the physical pin to see if signal is being received
 */

#include <Arduino.h>

#define FLOW_SENSOR_PIN 16

volatile unsigned long FlowPulseCount = 0;
volatile unsigned long LastPulseTime = 0;

void IRAM_ATTR FlowSensorISR_RISING() {
    FlowPulseCount++;
    LastPulseTime = millis();
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n=== FLOW SENSOR DIAGNOSTIC ===");
    Serial.println("Pin: " + String(FLOW_SENSOR_PIN));
    Serial.println("Testing for signal...\n");
    
    pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
    
    // Test 1: Read raw pin state
    Serial.println("TEST 1: Raw pin state (should toggle 0-1 if flowing)");
    delay(2000);
    
    // Attach interrupt
    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), FlowSensorISR_RISING, RISING);
    Serial.println("\nTEST 2: Interrupt on RISING edge");
    Serial.println("(waiting 10 seconds for water flow)\n");
}

void loop() {
    static unsigned long lastPrint = 0;
    
    // Print raw pin state every 100ms
    if (millis() - lastPrint > 100) {
        int pinState = digitalRead(FLOW_SENSOR_PIN);
        Serial.print("Pin State: ");
        Serial.print(pinState);
        Serial.print(" | Total Pulses: ");
        Serial.print(FlowPulseCount);
        Serial.print(" | Last Pulse: ");
        Serial.print(millis() - LastPulseTime);
        Serial.println("ms ago");
        lastPrint = millis();
    }
    
    // If no pulses after 10 seconds, suggest troubleshooting
    static unsigned long startTime = 0;
    if (startTime == 0) startTime = millis();
    
    if (millis() - startTime > 10000 && FlowPulseCount == 0) {
        Serial.println("\n!!! NO PULSES DETECTED !!!");
        Serial.println("Possible issues:");
        Serial.println("1. Sensor not connected to pin 16");
        Serial.println("2. Sensor not powered (check 5V/GND)");
        Serial.println("3. Water not flowing through sensor");
        Serial.println("4. Sensor wires swapped (signal/GND)");
        Serial.println("5. Pin 16 doesn't support interrupts on your board\n");
        
        // Keep printing to see if sensor ever wakes up
        delay(5000);
    }
}
