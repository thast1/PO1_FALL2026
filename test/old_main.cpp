/* 
 * System Control
 * This file contains the main control loop for the system, including:
 * - Reading sensors (weight, temperature, flow)
 * - Updating system state based on sensor readings
 * - Controlling actuators (motor, pump, solenoid, heater relay) based on state
 * Actuators:
 * - Motor: Controls mixing mechanism, ON/OFF control via GPIO
 * - Pump: Controls water flow, ON/OFF control via GPIO
 * - Solenoid: Controls valve, ON/OFF control via GPIO
 * - Heater Relay: Controls AC 120V heater via relay module, ON/OFF control via GPIO
 *   - SAFETY: Ensure proper relay module with optical isolation recommended
 *   - Add appropriate fusing on 120V AC side for fire safety
 *   - Do not connect 120V AC directly to ESP32 GPIO pins
 */

/* TO DO:
    - Pin Config
    - State Machines
    - Update Logic
    - Actuator Control
    - Wifi/App Communication
    - Timing & Safety
    - Sensor Reading Loop
*/

//======================================================================================
// LIBRARY INCLUDES
//======================================================================================
#include <Arduino.h> // Arduino Framework
#include "HX711.h" // HX711 ADC Signal Amplifier for Load Cells
#include <OneWire.h> // Dallas OneWire Communication Protocol
#include <DallasTemperature.h> // Dallas DS18B20 Temperature Sensor Interfacing Protocol

// USE THIS OR REST API??
// You'll need to add these libraries to your includes at the top:
// #include <WebServer.h>
// #include <WebSocketsServer.h>
// #include <ArduinoJson.h>

//======================================================================================
// GPIO PIN CONFIGURATION
//======================================================================================

// CONTROL PINS                 !!!!FIX PINS!!!!
#define MOTOR_PWM_PIN    18
#define MOTOR_DIR_PIN    12
#define PUMP_1_PIN       40
#define PUMP_2_PIN       41
#define HEATER_PIN        8
#define SOLENOID_PIN     42

// SENSOR PINS
#define TEMP_SENSOR_PIN 15
#define FLOW_SENSOR_PIN_1 16
#define FLOW_SENSOR_PIN_2 7
#define DOUT_1          17
#define CLK_1           20
#define DOUT_2          10
#define CLK_2           11

// CONFIGURE UNUSED PINS TO INTEGRATED RESISTORS

//======================================================================================
// GLOBAL VARIABLES
//======================================================================================

// STATE VARIABLES
unsigned long int StateChangeTime = 0;    // Initialize Time Tracking for States

// TARGET VALUES
unsigned int bean_weight_diff     = 0; // Target weight for grinding (current weight - target weight)
unsigned int water_temp           = 0; // Target temperature for heating water
unsigned int flow_rate            = 0; // Target flow rate for pumping water (mL/sec)

// CYCLE TRACKING
//Duration
unsigned long StateStartTime      = 0; // Time when the current state started
unsigned long StateDuration       = 0; // Time spent in the current state, updated continuously
unsigned int BootStep             = 0; // Track which boot step we're on
unsigned int BootRetries          = 0; // Track retries for current boot step
const unsigned int MAX_BOOT_RETRIES = 3; // Max retries per component

//Flags
struct STATE_FLAGS {
    //GENERAL
    bool GENERAL_Initialized         = false;    // State is being run for first time since power up
    //IDLE
    bool IDLE_SystemReady            = false;       // System has completed initialization and is ready for user input
    //GRIND
    bool GRIND_MotorStalled          = false;     // Motor stall detected during grinding (no weight change for certain time)
    bool GRIND_WeightMeasured        = false;    // Initial weight measured, container confirmed, motor started
    bool GRIND_WeightReached         = false;    // Target weight reached during grinding (weight change >= bean_weight_diff)
    bool GRIND_TimeoutOccurred       = false;  // Grinding has exceeded expected time without reaching target weight (potential stall or error)
    //USER_PROMPT
    bool USER_PromptAcknowledged     = false;
    bool USER_TimeoutOccurred        = false;
    //PUMP
    bool PUMP_FlowDetected           = false;
    bool PUMP_TimeoutOccurred        = false;     
    //HEAT  
    bool HEAT_TargetTempReached      = false;
    bool HEAT_OverheatTriggered      = false;
    bool HEAT_TimeoutOccurred        = false;   
    //DISPENSE  
    bool DISPENSE_DispensingComplete = false;
    bool DISPENSE_TimeoutOccurred    = false;
    //ERROR 
    bool ERROR_ErrorAcknowledged     = false;
    bool ERROR_Shutdown              = false;
};

STATE_FLAGS StateFlags;

// SAFETY
float MaxTemp = 100.0; // Max Temp in C
float MaxBeanWeight = 75.0; // in grams
float MaxWaterWeight = 1000.0; // in mL
unsigned long GlobalTimeout = 300000;

// SENSOR OBJECTS
// HX711/LoadCells
HX711 LoadCellBeans_1; // 4541 Load Cell
HX711 LoadCellBeans_2; // 4541 Load Cell
float GrindWeightBeans         = 0.0;
float weightGround             = 0.0;
float InitialWeightBeans       = 0.0; 
float CurrentWeightWater       = 0.0;
float Offset_1                 = 0.0;
float Offset_2                 = 0.0;
float CalibrationFactor_1      = 164.6 / 69800;
float CalibrationFactor_2      = 164.6 / 71600;

//DS18B20
OneWire OneWireInstance(TEMP_SENSOR_PIN);
DallasTemperature TempSensor(&OneWireInstance);
float CurrentTemperature     = 0.0;

//YF201 Flow Sensor
volatile unsigned long FlowPulseCount = 0;  // Interrupt-safe counter
float         CurrentFlowRate         = 0.0;        // mL/sec
unsigned long LastFlowMeasureTime     = 0;      // Last time flow was measured
unsigned long LastFlowPulseCount      = 0;       // Pulse count at last measurement
const float FLOW_SENSOR_CALIBRATION   = 0.6;  // YF201 pulses per mL (adjust based on calibration)

// APP COMMUNICATION
struct RecipeData {
    unsigned int TargetBeanWeight  =  25;  // grams
    unsigned int TargetWaterTemp   =  50;   // °C
    unsigned int TargetWaterWeight = 150; // mL
    unsigned int TargetFlowRate;    // mL/sec
};

RecipeData CurrentRecipe;
bool RecipeReceived = false;
bool StartCommandReceived = false;
bool UserAcknowledgmentReceived = false;
bool EmergencyStopReceived = false;

// Status to send back to app
struct SystemStatus {
    String CurrentState;
    float BeanWeight;
    float WaterWeight;
    float BoilerTemp;
    String ErrorMessage;
    bool IsRunning;
};

SystemStatus StatusToSend;

// WiFi Configuration
const char* WIFI_SSID     = "";        // Fill in your WiFi SSID
const char* WIFI_PASSWORD = "";    // Fill in your WiFi password
const int SERVER_PORT     = 80;   

// USE THIS OR REST API??
// You'll need to add these libraries to your includes at the top:
// #include <WebServer.h>
// #include <WebSocketsServer.h>
// #include <ArduinoJson.h>

// WebSocket server instance (declare globally after RecipeData)
// WebSocketsServer webSocket = WebSocketsServer(WEBSOCKET_PORT);

//======================================================================================
// INTERRUPT SERVICE ROUTINES
//======================================================================================

// YF201 Flow Sensor ISR - counts pulses from turbine
void IRAM_ATTR FlowSensorISR() {
    FlowPulseCount++;
}

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
// STATE MACHINE LOGIC
//======================================================================================

enum MachineStates{ // !!!!WRITE COMMENTS!!!!
    IDLE,           // Initialization State, machine will start and finish here.
    GRIND,          // Grind beans to target weight, monitor for motor stall and weight reached
    USER_PROMPT,    // Prompt user to move container, monitor for acknowledgment and timeout
    PUMP,           // Pump water for target time, monitor for flow and weight reached
    HEAT,           // Heat water to target temperature, monitor for overheat and sensor errors
    DISPENSE,       // Dispense coffee, monitor for completion and timeout
    ERROR           // Handle errors, monitor for acknowledgment and shutdown
};

MachineStates CurrentState = DISPENSE; // Initialize in IDLE

/*void HandleIDLE(){
    if (StateFlags.GENERAL_Initialized == false){
        if (StateFlags.IDLE_SystemReady == false) {
            // Boot sequence - test actuators and sensors
            if (BootStep == 0) { // Test Motor
                digitalWrite(MOTOR_DRIVER_PIN, HIGH);
                delay(100);
                digitalWrite(MOTOR_DRIVER_PIN, LOW);
                BootStep++;
                BootRetries = 0;
            }
            else if (BootStep == 1) { // Test Pump1
                digitalWrite(PUMP_1_PIN, HIGH);
                delay(100);
                digitalWrite(PUMP_1_PIN, LOW);
                BootStep++;
                BootRetries = 0;
            }
            else if (BootStep == 2) { // Test Pump2
                digitalWrite(PUMP_2_PIN, HIGH);
                delay(100);
                digitalWrite(PUMP_2_PIN, LOW);
                BootStep++;
                BootRetries = 0;
            }
            else if (BootStep == 3) { // Test Heater
                digitalWrite(HEATER_PIN, HIGH);
                delay(100);
                digitalWrite(HEATER_PIN, LOW);
                BootStep++;
                BootRetries = 0;
            }
            else if (BootStep == 4) { // Test Solenoid
                digitalWrite(SOLENOID_PIN, HIGH);
                delay(100);
                digitalWrite(SOLENOID_PIN, LOW);
                BootStep++;
                BootRetries = 0;
            }
            else if (BootStep == 5) { // Read LoadCellBeans
                CurrentWeightBeans = LoadCellBeans.get_units();
                if (CurrentWeightBeans > -1000 && CurrentWeightBeans < 1000) {
                    BootStep++;
                    BootRetries = 0;
                } else {
                    BootRetries++;
                    if (BootRetries > MAX_BOOT_RETRIES) {
                        CurrentState = ERROR;
                        StateFlags.IDLE_SystemReady = true;
                    }
                }
            }
            else if (BootStep == 6) { // Calibrate LoadCellBeans
                CalibrationFactorBeans = 1.0;
                BootStep++;
                BootRetries = 0;
            }
            else if (BootStep == 7) { // Read LoadCellWater
                CurrentWeightWater = LoadCellWater.get_units();
                if (CurrentWeightWater > -1000 && CurrentWeightWater < 2000) {
                    BootStep++;
                    BootRetries = 0;
                } else {
                    BootRetries++;
                    if (BootRetries > MAX_BOOT_RETRIES) {
                        CurrentState = ERROR;
                        StateFlags.IDLE_SystemReady = true;
                    }
                }
            }
            else if (BootStep == 8) { // Calibrate LoadCellWater
                CalibrationFactorWater = 1.0;
                BootStep++;
                BootRetries = 0;
            }
            else if (BootStep == 9) { // Read TempSensor
                TempSensor.requestTemperatures();
                CurrentTemperature = TempSensor.getTempCByIndex(0);
                if (CurrentTemperature > -50 && CurrentTemperature < 150) {
                    BootStep++;
                    BootRetries = 0;
                } else {
                    BootRetries++;
                    if (BootRetries > MAX_BOOT_RETRIES) {
                        CurrentState = ERROR;
                        StateFlags.IDLE_SystemReady = true;
                    }
                }
            }
            else if (BootStep == 10) { // All tests passed
                StateFlags.IDLE_SystemReady = true;
                StateFlags.GENERAL_Initialized = true;
                BootStep = 0;
                BootRetries = 0;
            }
            
            // Check boot timeout
            if (millis() - StateStartTime > 60000) {
                CurrentState = ERROR;
                StateFlags.IDLE_SystemReady = true;
            }
        }
    }
    else {
        // Already initialized - ready state
        // Ensure all actuators are OFF (safe state)
        digitalWrite(MOTOR_DRIVER_PIN, LOW);
        digitalWrite(PUMP_1_PIN, LOW);
        digitalWrite(PUMP_2_PIN, LOW);
        digitalWrite(HEATER_PIN, LOW);
        digitalWrite(SOLENOID_PIN, LOW);
        
        // Wait for recipe from app
        if (RecipeReceived == true){
            CurrentRecipe.TargetBeanWeight = bean_weight_diff;
            CurrentRecipe.TargetWaterTemp = water_temp;
            CurrentRecipe.TargetWaterWeight = 0;
            CurrentRecipe.TargetFlowRate = flow_rate;
            
            CurrentState = GRIND;
            StateStartTime = millis();
            memset(&StateFlags, 0, sizeof(StateFlags));
            RecipeReceived = false;
        }
    }
}
*/
void HandleTARE(){
    delay(60000);
    LoadCellBeans_1.begin(DOUT_1, CLK_1);
    LoadCellBeans_2.begin(DOUT_2, CLK_2);

    // Confirm both sensors visible before running, if not ready delay start
    unsigned long start = millis();
    while ((!LoadCellBeans_1.is_ready() || !LoadCellBeans_2.is_ready()) &&
           (millis() - start < 5000)) {
        delay(50);
    }

    long sum_1 = 0;
    long sum_2 = 0;


    if (LoadCellBeans_1.is_ready() && LoadCellBeans_2.is_ready()) {

        // Take 25 readings to estimate offset
        for (int i = 1; i < 26; i++){
            long reading_1 = LoadCellBeans_1.read();
            long reading_2 = LoadCellBeans_2.read();
            
            sum_1 += reading_1;
            sum_2 += reading_2;
            delay(200);
        }    
    } 
    else{
        Serial.println("One or both HX711s not ready.");
    }

    // Calculate Average Offset
    Offset_1 = sum_1 / 25.0;
    Offset_2 = sum_2 / 25.0;

    Serial.print("HX711_1 CF: ");
    Serial.println(Offset_1);
    Serial.print("HX711_2 CF: ");
    Serial.println(Offset_2);
}
void HandleGRIND(){
    // PHASE 1: Wait for container to be placed on scale
    if (StateFlags.GENERAL_Initialized == false){ // First run this state
        pinMode(MOTOR_PWM_PIN, OUTPUT);
        pinMode(MOTOR_DIR_PIN, OUTPUT);

        Serial.println("[GRIND] Place container on scale and press 's' to start");
        StateStartTime = millis();
        StateFlags.GENERAL_Initialized = true;
        return;  // Exit early, wait for input
    }
    
    // PHASE 2: Wait for user confirmation, then measure initial weight and start motor
    if (StateFlags.GRIND_WeightMeasured == false) {
        if (Serial.available()) {
            char input = Serial.read();
            if (input == 's' || input == 'S') {
                Serial.println("[GRIND] Container confirmed. Measuring initial weight...");
            
                StateStartTime = millis();
                InitialWeightBeans = (LoadCellBeans_1.read() - Offset_1) * CalibrationFactor_1 + 
                                     (LoadCellBeans_2.read() - Offset_2) * CalibrationFactor_2;
                GrindWeightBeans = InitialWeightBeans;

                Serial.println("[GRIND TEST] Initial weight recorded: " + String(InitialWeightBeans)); //TEST
                Serial.println("[GRIND TEST] Target weight to grind: " + String(CurrentRecipe.TargetBeanWeight));
        
                // Turn motor ON
                digitalWrite(MOTOR_PWM_PIN, HIGH);
                digitalWrite(MOTOR_DIR_PIN, LOW);
                Serial.println("[GRIND TEST] Motor turned ON"); //TEST
                StateFlags.GRIND_WeightMeasured = true;
                return;
            }
        }
        
        // Check timeout while waiting for container confirmation (5 minutes)
        if (millis() - StateStartTime > 300000) {
            Serial.println("[GRIND] TIMEOUT waiting for container confirmation");
            CurrentState = ERROR;
            StateStartTime = millis();
            memset(&StateFlags, 0, sizeof(StateFlags));
            return;
        }
        return;  // Keep waiting for input
    }
    
    // PHASE 3: Normal grinding operation (motor running, monitoring weight)
    
    // Check timeout for safety
    if (millis() - StateStartTime > 60000){ // 60 second timeout
        StateFlags.GRIND_TimeoutOccurred = true;
        digitalWrite(MOTOR_PWM_PIN, LOW);  // Turn motor OFF
        Serial.println("[GRIND TEST] TIMEOUT - Motor turned OFF"); //TEST
        CurrentState = ERROR;
        StateStartTime = millis();
        memset(&StateFlags, 0, sizeof(StateFlags));
        return;
    }
    
    // Switch motor direction every 2 seconds
    static unsigned long lastDirectionSwitch = 0;
    unsigned long elapsedSinceInit = millis() - StateStartTime;
    unsigned long directionCycle = elapsedSinceInit / 5000;  // Switch every 5000ms
    
    if (millis() - lastDirectionSwitch > 5000){
        // Alternate direction: even cycles = LOW, odd cycles = HIGH
        if (directionCycle % 2 == 0){
            delay(2000);
            digitalWrite(MOTOR_DIR_PIN, LOW);
        } else
         {
            delay(2000);
            digitalWrite(MOTOR_DIR_PIN, HIGH);
        }
        lastDirectionSwitch = millis();
    }
    
    // Measure weight periodically (every 100ms)
    static unsigned long lastMeasureTime = 0;
    if (millis() - lastMeasureTime > 100){
        GrindWeightBeans = (LoadCellBeans_1.read() - Offset_1) * CalibrationFactor_1 + 
                           (LoadCellBeans_2.read() - Offset_2) * CalibrationFactor_2;
        lastMeasureTime = millis();
        
        // Check if target weight reached
        weightGround = GrindWeightBeans - InitialWeightBeans;
        Serial.println("[GRIND TEST] Current ground weight: " + String(weightGround) + "g");
        if (weightGround >= CurrentRecipe.TargetBeanWeight){
            Serial.println("[GRIND TEST] TARGET REACHED!");
            Serial.println("[GRIND TEST] GRIND_WeightReached flag: TRUE");
            Serial.println("[GRIND TEST] Motor turned OFF");
            Serial.println("[GRIND TEST] State transitioning to USER_PROMPT");
            StateFlags.GRIND_WeightReached = true;
            digitalWrite(MOTOR_PWM_PIN, LOW);  // Turn motor OFF
            CurrentState = USER_PROMPT;
            StateStartTime = millis();
            memset(&StateFlags, 0, sizeof(StateFlags));
            lastMeasureTime = 0; // Reset for next state
            lastDirectionSwitch = 0; // Reset for next state
            return;
        }
    }
}
void HandleUSER_PROMPT(){
    if (StateFlags.GENERAL_Initialized == false){ // First run this state
        pinMode(PUMP_1_PIN, OUTPUT);
        pinMode(FLOW_SENSOR_PIN_1, INPUT_PULLUP);

        Serial.println("[USER_PROMPT] Move container to dispenser and press 'c' to confirm");
        StateStartTime = millis();
        StateDuration = 120000;  // 2 minute max wait for user confirmation
        
        // Setup flow sensor interrupt
        attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN_1), FlowSensorISR, RISING);
        
        // Reset flow tracking variables
        FlowPulseCount = 0;
        LastFlowPulseCount = 0;
        LastFlowMeasureTime = millis();
        
        StateFlags.GENERAL_Initialized = true;
    }
    
    // PHASE 1: Wait for user confirmation
    if (StateFlags.USER_PromptAcknowledged == false) {
        // Check for serial input
        if (Serial.available()) {
            char input = Serial.read();
            if (input == 'c' || input == 'C') {
                StateFlags.USER_PromptAcknowledged = true;
                
                // Start PUMP_1 (fill boiler)
                digitalWrite(PUMP_1_PIN, HIGH);
                Serial.println("[USER_PROMPT] Container confirmed. Pump started - filling boiler...");
                Serial.println("[USER_PROMPT] Target water weight: " + String(CurrentRecipe.TargetWaterWeight) + " mL");
                
                // Reset pump timer
                StateStartTime = millis();
                CurrentWeightWater = 0.0;
                return;
            }
        }
        
        // Check for timeout waiting for confirmation
        if (millis() - StateStartTime > StateDuration){
            StateFlags.USER_TimeoutOccurred = true;
            Serial.println("[USER_PROMPT] TIMEOUT waiting for user confirmation");
            detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN_1));
            CurrentState = ERROR;
            StateStartTime = millis();
            memset(&StateFlags, 0, sizeof(StateFlags));
            return;
        }
        return;  // Keep waiting for confirmation
    }
    
    // PHASE 2: Pump until target water weight reached
    
    // Measure flow rate and accumulate water weight (every 100ms)
    static unsigned long lastFlowCheck = 0;
    if (millis() - lastFlowCheck > 100) {
        CurrentFlowRate = CalculateFlowRate();
        lastFlowCheck = millis();
        
        // Calculate water accumulated since pump started
        // Volume = (pulses / calibration_factor)
        CurrentWeightWater = (float)FlowPulseCount / FLOW_SENSOR_CALIBRATION;
        
        Serial.print("[USER_PROMPT] Flow Rate: ");
        Serial.print(CurrentFlowRate);
        Serial.print(" mL/sec | Water accumulated: ");
        Serial.print(CurrentWeightWater);
        Serial.println(" mL");
        
        // Detect if flow is present (threshold: > 0.1 mL/sec)
        if (CurrentFlowRate > 0.1 && StateFlags.PUMP_FlowDetected == false) {
            StateFlags.PUMP_FlowDetected = true;
            Serial.println("[USER_PROMPT] Flow detected!");
        }
        
        // Check if flow stops unexpectedly (after being detected)
        if (CurrentFlowRate < 0.05 && StateFlags.PUMP_FlowDetected == true && millis() > 1000) {
            StateFlags.PUMP_TimeoutOccurred = true;
            Serial.println("[USER_PROMPT] Flow stopped unexpectedly!");
            digitalWrite(PUMP_1_PIN, LOW);
            detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN_1));
            CurrentState = ERROR;
            StateStartTime = millis();
            memset(&StateFlags, 0, sizeof(StateFlags));
            return;
        }
    }
    
    // Check if target water weight reached
    if (CurrentWeightWater >= CurrentRecipe.TargetWaterWeight){
        Serial.println("[USER_PROMPT] Target water weight reached!");
        Serial.println("[USER_PROMPT] Pump turned OFF");
        Serial.println("[USER_PROMPT] State transitioning to HEAT");
        
        digitalWrite(PUMP_1_PIN, LOW);  // Turn pump OFF
        detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN_1));  // Disable flow sensor interrupt
        FlowPulseCount = 0;  // Reset pulse count
        
        CurrentState = HEAT;
        StateStartTime = millis();
        memset(&StateFlags, 0, sizeof(StateFlags));
        return;
    }
    
    // Check for timeout during pumping (5 minutes)
    if (millis() - StateStartTime > 300000){
        StateFlags.PUMP_TimeoutOccurred = true;
        Serial.println("[USER_PROMPT] TIMEOUT during pumping");
        digitalWrite(PUMP_1_PIN, LOW);  // Turn PUMP_1 OFF
        detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN_1));
        CurrentState = ERROR;
        StateStartTime = millis();
        memset(&StateFlags, 0, sizeof(StateFlags));
        return;
    }
}
void HandleHEAT(){
    if (StateFlags.GENERAL_Initialized == false){ // First run this state
        digitalWrite(HEATER_PIN, HIGH);  // Turn heater ON
        StateStartTime = millis();
        StateDuration  = 600000;  // 10 minute max heating time
        StateFlags.GENERAL_Initialized = true;
    }
    
    // Read current water temperature
    TempSensor.requestTemperatures();
    CurrentTemperature = TempSensor.getTempCByIndex(0);
    Serial.print(CurrentTemperature);
    Serial.println(" C");
    
    // Check if target temperature reached
    if (CurrentTemperature >= CurrentRecipe.TargetWaterTemp){
        StateFlags.HEAT_TargetTempReached = true;
        digitalWrite(HEATER_PIN, LOW);  // Turn heater OFF
        Serial.print("[HEAT] Target Temperature Reached. Transitioned to [DISPENSE]");
        CurrentState   = DISPENSE;
        StateStartTime = millis();
        memset(&StateFlags, 0, sizeof(StateFlags));
        return;
    }
    
    // Check for overheat (safety limit)
    if (CurrentTemperature > MaxTemp){
        StateFlags.HEAT_OverheatTriggered = true;
        digitalWrite(HEATER_PIN, LOW);  // Turn heater OFF
        Serial.print("[HEAT] Cartridge Exceeded Max Temperature.");
        CurrentState   = ERROR;
        StateStartTime = millis();
        memset(&StateFlags, 0, sizeof(StateFlags));
        return;
    }
    
    // Check for timeout
    if (millis() - StateStartTime > StateDuration){
        StateFlags.HEAT_TimeoutOccurred = true;
        digitalWrite(HEATER_PIN, LOW);  // Turn heater OFF
        Serial.print("[HEAT] Timeout Occured.");
        CurrentState = ERROR;
        StateStartTime = millis();
        memset(&StateFlags, 0, sizeof(StateFlags));
        return;
    }
}
void HandleDISPENSE(){


    // 45 ML IN SHOWER HEAD MAX.
    unsigned long elapsedTime = millis() - StateStartTime;
    float bloomVolume = weightGround * 2;
    float remainingVolume = CurrentWeightWater - bloomVolume;
    float volumePerCycle = remainingVolume / 4;

    if (StateFlags.GENERAL_Initialized == false){ // First run this state
        pinMode(SOLENOID_PIN, OUTPUT);
        pinMode(PUMP_2_PIN, OUTPUT);
        pinMode(FLOW_SENSOR_PIN_2, INPUT_PULLUP);
        
        // Setup flow sensor interrupt for dispense
        attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN_2), FlowSensorISR, RISING);
        
        // Reset flow tracking variables
        FlowPulseCount = 0;
        LastFlowPulseCount = 0;
        LastFlowMeasureTime = millis();
        
        digitalWrite(SOLENOID_PIN, HIGH); // Turn solenoid ON (open valve)
        delay(500);
        digitalWrite(PUMP_2_PIN, HIGH);   // Turn PUMP_2 ON
        
        StateStartTime = millis();
        StateDuration = 270000;  // 4:30 minutes total (270 seconds)
        
        Serial.println("[DISPENSE] Starting bloom phase...");
        Serial.print("[DISPENSE] Bean weight: ");
        Serial.print(weightGround);
        Serial.println(" g");
        Serial.print("[DISPENSE] Bloom volume: ");
        Serial.print(bloomVolume);
        Serial.println(" mL");
        Serial.print("[DISPENSE] Water available: ");
        Serial.print(CurrentWeightWater);
        Serial.println(" mL");
        Serial.print("[DISPENSE] Remaining volume per cycle: ");
        Serial.print(volumePerCycle);
        Serial.println(" mL");
        
        StateFlags.GENERAL_Initialized = true;
        return;
    }
    
   
    
    // Time allocation: 45s bloom + 4 cycles with pauses
    const unsigned long bloomDuration = 10000;      // 10 seconds
    const unsigned long timePerCycle = 5000;       // ~56 seconds per cycle
    const unsigned long pauseBetweenCycles = 30000; // 30 second pause
    
    // Calculate total water dispensed so far
    float totalWaterDispensed = (float)FlowPulseCount / FLOW_SENSOR_CALIBRATION;
    
    // PHASE 1: BLOOM (0-45 seconds)
    if (elapsedTime < bloomDuration) {
        // Monitor bloom phase
        CurrentFlowRate = CalculateFlowRate();
        
        Serial.print("[DISPENSE] BLOOM Phase - Elapsed: ");
        Serial.print(elapsedTime / 1000);
        Serial.print("s | Flow: ");
        Serial.print(CurrentFlowRate);
        Serial.print(" mL/s | Dispensed: ");
        Serial.print(totalWaterDispensed);
        Serial.print(" / ");
        Serial.print(bloomVolume);
        Serial.println(" mL");
        
        // Safety check: if bloom phase overflows
        if (totalWaterDispensed > bloomVolume * 1.1) {
            Serial.println("[DISPENSE] WARNING: Bloom phase exceeded target volume!");
        }
        return;
    }
    
    // PHASES 2-5: FOUR DISPENSE CYCLES (after bloom)
    unsigned long timeIntoCycles = elapsedTime - bloomDuration;
    unsigned long totalCycleDuration = timePerCycle + pauseBetweenCycles;
    
    // Determine which cycle we're in (0-3, or 4 if all complete)
    unsigned int currentCycle = (timeIntoCycles / totalCycleDuration) + 1;
    unsigned long timeIntoCurrentCycle = timeIntoCycles % totalCycleDuration;
    
    // All cycles complete
    if (currentCycle > 4) {
        digitalWrite(PUMP_2_PIN, LOW);
        digitalWrite(SOLENOID_PIN, LOW);
        detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN_2));
        
        Serial.println("[DISPENSE] ========== DISPENSE COMPLETE ==========");
        Serial.print("[DISPENSE] Total water dispensed: ");
        Serial.print(totalWaterDispensed);
        Serial.println(" mL");
        Serial.print("[DISPENSE] Total time: ");
        Serial.print(elapsedTime / 1000);
        Serial.println(" seconds");
        
        StateFlags.DISPENSE_DispensingComplete = true;
        CurrentState = IDLE;
        StateStartTime = millis();
        memset(&StateFlags, 0, sizeof(StateFlags));
        FlowPulseCount = 0;  // Reset pulse count
        return;
    }
    
    // During active dispense cycle (not pause)
    if (timeIntoCurrentCycle < timePerCycle) {
        CurrentFlowRate = CalculateFlowRate();
        
        // Calculate water dispensed in this cycle
        // Total dispensed minus bloom and previous cycles
        float waterInPreviousCycles = (currentCycle - 1) * volumePerCycle;
        float waterInThisCycle = totalWaterDispensed - bloomVolume - waterInPreviousCycles;
        
        Serial.print("[DISPENSE] CYCLE ");
        Serial.print(currentCycle);
        Serial.print(" - Elapsed in cycle: ");
        Serial.print(timeIntoCurrentCycle / 1000);
        Serial.print("s | Flow: ");
        Serial.print(CurrentFlowRate);
        Serial.print(" mL/s | Cycle progress: ");
        Serial.print(waterInThisCycle);
        Serial.print(" / ");
        Serial.print(volumePerCycle);
        Serial.println(" mL");
        
        // Safety check: detect if flow stopped during active phase
        if (CurrentFlowRate < 0.05 && timeIntoCurrentCycle > 5000) {
            Serial.println("[DISPENSE] ERROR: Flow stopped during active cycle!");
            digitalWrite(PUMP_2_PIN, LOW);
            digitalWrite(SOLENOID_PIN, LOW);
            detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN_2));
            
            StateFlags.DISPENSE_TimeoutOccurred = true;
            CurrentState = ERROR;
            StateStartTime = millis();
            memset(&StateFlags, 0, sizeof(StateFlags));
            FlowPulseCount = 0;
            return;
        }
    } 
    else {
        // Pause between cycles
        unsigned long timeIntoPause = timeIntoCurrentCycle - timePerCycle;
        
        Serial.print("[DISPENSE] PAUSE after cycle ");
        Serial.print(currentCycle);
        Serial.print(" - Elapsed: ");
        Serial.print(timeIntoPause / 1000);
        Serial.print("s / 30s");
        Serial.println();
    }
    
    // Safety timeout check (with 2 second margin)
    if (elapsedTime > StateDuration + 2000) {
        StateFlags.DISPENSE_TimeoutOccurred = true;
        digitalWrite(PUMP_2_PIN, LOW);
        digitalWrite(SOLENOID_PIN, LOW);
        detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN_2));
        
        Serial.println("[DISPENSE] TIMEOUT - Stopping dispense");
        CurrentState = ERROR;
        StateStartTime = millis();
        memset(&StateFlags, 0, sizeof(StateFlags));
        FlowPulseCount = 0;
        return;
    }
}
void HandleERROR(){ //NOT CORRECT CURRENTLY
    // Turn ALL actuators OFF (safety shutdown)
    //digitalWrite(MOTOR_DRIVER_PIN, LOW);
    digitalWrite(PUMP_1_PIN, LOW);
    digitalWrite(PUMP_2_PIN, LOW);
    digitalWrite(HEATER_PIN, LOW);
    digitalWrite(SOLENOID_PIN, LOW);
    
    // Send error message to app (placeholder)
    // SendErrorToApp(StateFlags);
    
    // Check for user acknowledgment
    if (UserAcknowledgmentReceived == true){
        StateFlags.ERROR_ErrorAcknowledged = true;
        CurrentState = IDLE;
        StateStartTime = millis();
        memset(&StateFlags, 0, sizeof(StateFlags));
        UserAcknowledgmentReceived = false;  // Reset flag
        BootStep = 0;  // Reset boot sequence
        BootRetries = 0;
        return;
    }
    
    // Check for timeout (10 minutes)
    if (millis() - StateStartTime > 600000){
        StateFlags.ERROR_Shutdown = true;
        // System halt or deep sleep
    }
}

//======================================================================================
// IMPLEMENTATION LOGIC
//======================================================================================

void setup() {
    Serial.begin(115200);
    TempSensor.begin();
 //   pinMode(HEATER_PIN, OUTPUT);
  //  digitalWrite(HEATER_PIN, LOW);
   // HandleTARE();

   pinMode(SOLENOID_PIN, OUTPUT);
        pinMode(PUMP_2_PIN, OUTPUT);
        pinMode(FLOW_SENSOR_PIN_2, INPUT_PULLUP);

   digitalWrite(SOLENOID_PIN, HIGH); // Turn solenoid ON (open valve)
        delay(500);
        digitalWrite(PUMP_2_PIN, HIGH);   // Turn PUMP_2 ON
    
}

void loop(){  


    /*switch(CurrentState){
        case IDLE:
            //HandleIDLE();
            break;
        case GRIND:
            HandleGRIND();
            break;
        case USER_PROMPT:
            HandleUSER_PROMPT();
            break;
        case HEAT:
            HandleHEAT();
            break;
        case DISPENSE:
            //HandleDISPENSE();
            break;
        case ERROR:
            //HandleERROR();
            break;
    }
            */
}
