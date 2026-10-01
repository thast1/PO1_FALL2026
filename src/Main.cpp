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

// App communication over Bluetooth - same approach as PO1_Hardware_BLE_Test
// (standalone BLE proof-of-concept) in the MobileApp repo.
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>

// USE THIS OR REST API??
// You'll need to add these libraries to your includes at the top:
// #include <WebServer.h>
// #include <WebSocketsServer.h>

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
unsigned int water_weight         = 0; // Target water weight, staged from the app's SET_RECIPE command
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

// DISPENSE / SHOWERHEAD LIMITS
const float SHOWERHEAD_CAPACITY_ML = 45.0;
const unsigned long SHOWERHEAD_DRAIN_TIME_MS = 10000;
const unsigned long BLOOM_SOAK_TIME_MS = 45000;
const unsigned long PAUSE_BETWEEN_POURS_MS = 30000;
const unsigned long FILL_FLOW_TIMEOUT_MS = 60000;
const unsigned long FLOW_START_TIMEOUT_MS = 5000;

// APP COMMUNICATION
struct RecipeData {
    unsigned int TargetBeanWeight  =  25;  // grams
    unsigned int TargetWaterTemp   =  35;   // °C
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
    FlowPulseCount = FlowPulseCount + 1;
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

MachineStates CurrentState = IDLE; // Initialize in IDLE

// The dispenser fills the showerhead in 45 mL-or-smaller batches, then lets it drain.
enum DispensePhase {
    DISPENSE_BLOOM,
    DISPENSE_FILL_SHOWERHEAD,
    DISPENSE_DRAIN_SHOWERHEAD,
    DISPENSE_BLOOM_SOAK,
    DISPENSE_PAUSE_BETWEEN_POURS,
    DISPENSE_COMPLETE
};

DispensePhase CurrentDispensePhase = DISPENSE_BLOOM;
unsigned int CurrentPour = 0;  // 0 = bloom, 1-4 = regular pours
float TotalDispenseTargetML = 0.0;
float BloomTargetML = 37.5;
float PourTargetML = 0.0;
float CurrentPourTargetML = 0.0;
float CurrentPourDispensedML = 0.0;
float CurrentFillTargetML = 0.0;
unsigned long FillStartPulseCount = 0;
unsigned long DispensePhaseStartTime = 0;

//======================================================================================
// BLE COMMUNICATION (app <-> machine)
//======================================================================================
// Same device name + UUIDs as the BLE test rig in the MobileApp repo
// (PO1_Hardware_BLE_Test/src/main.cpp), so the app connects with no changes
// on its side.
#define BLE_SERVICE_UUID      "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_STATUS_CHAR_UUID  "6e400003-b5a3-f393-e0a9-e50e24dcca9e" // NOTIFY: machine -> app
#define BLE_COMMAND_CHAR_UUID "6e400002-b5a3-f393-e0a9-e50e24dcca9e" // WRITE:  app -> machine

BLEServer *pBleServer = nullptr;
BLECharacteristic *pBleStatusChar = nullptr;
BLECharacteristic *pBleCommandChar = nullptr;

// IMPORTANT: BLE callbacks (onWrite/onConnect/onDisconnect) run on the
// Bluetooth stack's own task, which has a small fixed stack. Calling
// notify()/startAdvertising() directly from inside a callback can overflow
// that stack and crash the board (confirmed on the BLE test rig). These
// flags let the callbacks just record "something happened" - the real BLE
// work happens in loop(), which runs on the main task with a much bigger
// stack.
volatile bool BleStatusUpdatePending = false;
volatile bool BleAdvertisingRestartPending = false;

class BleServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    Serial.println("[BLE] App connected");
  }
  void onDisconnect(BLEServer *server) override {
    Serial.println("[BLE] App disconnected");
    BleAdvertisingRestartPending = true; // so the app/dashboard can reconnect
  }
};

class BleCommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    String value = String(characteristic->getValue().c_str());
    Serial.print("[BLE] Command received: ");
    Serial.println(value);

    if (value.startsWith("{")) {
      // SET_RECIPE payload from sendRecipeToMachine() in the app's
      // machine.ts, e.g. {"cmd":"SET_RECIPE","tempC":92,"beanWeight":25,"waterWeight":150}
      StaticJsonDocument<128> doc;
      if (deserializeJson(doc, value) == DeserializationError::Ok &&
          doc["cmd"] == "SET_RECIPE") {
        water_temp = doc["tempC"] | water_temp;
        bean_weight_diff = doc["beanWeight"] | bean_weight_diff;
        water_weight = doc["waterWeight"] | water_weight;
        RecipeReceived = true;
      }
    } else if (value == "START_GRIND") {
      // Sent when the app's "Continue to Brew" is tapped on the grinder
      // screen. Satisfies the gate in HandleGRIND() below.
      StartCommandReceived = true;
    } else if (value == "START_DISPENSE") {
      // Sent when the app's "Continue to Brew" is tapped on the "Move
      // Filtered Cup" screen - that tap IS the cup-moved-to-dispenser
      // confirmation, so this satisfies the gate in HandleUSER_PROMPT()
      // below (not a separate "cup moved" command - the app only ever
      // sends this one at that point in the flow).
      UserAcknowledgmentReceived = true;
    } else if (value == "EMERGENCY_STOP") {
      // NOTE: this only sets the flag for now. HandleERROR() is marked
      // "NOT CORRECT CURRENTLY" and its case in loop() is still commented
      // out, so this does not yet force actuators off or change state -
      // that needs HandleERROR() finished and enabled first.
      EmergencyStopReceived = true;
    }

    BleStatusUpdatePending = true;
  }
};

// Turns the current MachineStates enum value into the string the app
// expects in its "status" field.
const char *BleStateName(MachineStates state) {
  switch (state) {
    case IDLE: return "IDLE";
    case GRIND: return "GRIND";
    case USER_PROMPT: return "USER_PROMPT";
    case PUMP: return "PUMP";
    case HEAT: return "HEAT";
    case DISPENSE: return "DISPENSE";
    case ERROR: return "ERROR";
  }
  return "UNKNOWN";
}

void BleSendStatusUpdate() {
  StaticJsonDocument<256> doc;
  doc["status"] = BleStateName(CurrentState);
  // DS18B20 reads Celsius; the app's screens are labelled "°F".
  doc["boilerTemp"] = (int)round(CurrentTemperature * 9.0 / 5.0 + 32.0);
  // No cup/tank sensors on this build yet - intentionally omitted rather
  // than faked. The app treats a missing cupPresent as "cup present".

  String json;
  serializeJson(doc, json);

  pBleStatusChar->setValue(json.c_str());
  pBleStatusChar->notify();
}

void BleSetup() {
  BLEDevice::init("PourOver1-BLE-Test"); // must match exactly - the app scans for this name
  pBleServer = BLEDevice::createServer();
  pBleServer->setCallbacks(new BleServerCallbacks());

  BLEService *pService = pBleServer->createService(BLE_SERVICE_UUID);

  pBleStatusChar = pService->createCharacteristic(
      BLE_STATUS_CHAR_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pBleStatusChar->addDescriptor(new BLE2902());

  pBleCommandChar = pService->createCharacteristic(
      BLE_COMMAND_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE);
  pBleCommandChar->setCallbacks(new BleCommandCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("[BLE] Advertising as 'PourOver1-BLE-Test'");
}

// Replaces the boot-self-test HandleIDLE() that used to live here
// (commented out) - it referenced hardware that isn't in this version of
// the file any more (MOTOR_DRIVER_PIN, a single LoadCellBeans/
// CalibrationFactorBeans, and a LoadCellWater/CalibrationFactorWater -
// water is measured by the flow sensor now, not a load cell). Rebuilding a
// boot self-test is worth doing once the water-sensing hardware is
// settled; this just waits for the app's recipe, same as the old one did
// in its "already initialized" branch.
void HandleIDLE(){
    // Safe state while we wait for the app to send a recipe.
    digitalWrite(MOTOR_PWM_PIN, LOW);
    digitalWrite(PUMP_1_PIN, LOW);
    digitalWrite(PUMP_2_PIN, LOW);
    digitalWrite(HEATER_PIN, LOW);
    digitalWrite(SOLENOID_PIN, LOW);

    if (RecipeReceived) {
        CurrentRecipe.TargetBeanWeight = bean_weight_diff;
        CurrentRecipe.TargetWaterTemp = water_temp;
        CurrentRecipe.TargetWaterWeight = water_weight;
        CurrentRecipe.TargetFlowRate = flow_rate;
        RecipeReceived = false;

        Serial.println("[IDLE] Recipe received, moving to GRIND");
        CurrentState = GRIND;
        StateStartTime = millis();
        memset(&StateFlags, 0, sizeof(StateFlags));
    }
}

void HandleTARE(){
    delay(5000);
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
        bool confirmed = false;
        if (Serial.available()) {
            char input = Serial.read();
            if (input == 's' || input == 'S') confirmed = true;
        }
        if (StartCommandReceived) {   // set by onWrite() for the app's "START_GRIND"
            StartCommandReceived = false;
            confirmed = true;
        }
        if (confirmed) {
                Serial.println("[GRIND] Container confirmed. Measuring initial weight...");
            
                StateStartTime = millis();
                InitialWeightBeans = (LoadCellBeans_1.read() - Offset_1) * abs(CalibrationFactor_1) + 
                                     (LoadCellBeans_2.read() - Offset_2) * abs(CalibrationFactor_2);
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
        bool cupConfirmed = false;
        // Check for serial input
        if (Serial.available()) {
            char input = Serial.read();
            if (input == 'c' || input == 'C') cupConfirmed = true;
        }
        if (UserAcknowledgmentReceived) {   // set by onWrite() for the app's "START_DISPENSE"
            UserAcknowledgmentReceived = false;
            cupConfirmed = true;
        }
        if (cupConfirmed) {
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

void StartShowerheadFill(float requestedVolumeML) {
    CurrentFillTargetML = requestedVolumeML > SHOWERHEAD_CAPACITY_ML
                              ? SHOWERHEAD_CAPACITY_ML
                              : requestedVolumeML;
    FillStartPulseCount = FlowPulseCount;
    DispensePhaseStartTime = millis();
    CurrentDispensePhase = DISPENSE_FILL_SHOWERHEAD;

    // Keep the valve open while pumping. It stays open during the subsequent drain.
    digitalWrite(SOLENOID_PIN, HIGH);
    digitalWrite(PUMP_2_PIN, HIGH);
}
void StopDispenseWithError(const char *message) {
    digitalWrite(PUMP_2_PIN, LOW);
    digitalWrite(SOLENOID_PIN, LOW);
    detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN_2));

    StateFlags.DISPENSE_TimeoutOccurred = true;
    StateFlags.GENERAL_Initialized = false;
    CurrentState = ERROR;
    StateStartTime = millis();
    Serial.println(message);
}
void HandleDISPENSE(){
    if (StateFlags.GENERAL_Initialized == false) { // First run in this state
        pinMode(SOLENOID_PIN, OUTPUT);
        pinMode(PUMP_2_PIN, OUTPUT);
        pinMode(FLOW_SENSOR_PIN_2, INPUT_PULLUP);

        FlowPulseCount = 0;
        LastFlowPulseCount = 0;
        LastFlowMeasureTime = millis();
        attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN_2), FlowSensorISR, RISING);

        // Prefer the requested recipe volume. The measured boiler volume is a fallback
        // for the current standalone test configuration.
        TotalDispenseTargetML = CurrentRecipe.TargetWaterWeight > 0
                                    ? CurrentRecipe.TargetWaterWeight
                                    : CurrentWeightWater;
        if (TotalDispenseTargetML <= 0.0) {
            StopDispenseWithError("[DISPENSE] ERROR: No target water volume available");
            return;
        }

        BloomTargetML = weightGround * 2.0;
        if (BloomTargetML > TotalDispenseTargetML) {
            BloomTargetML = TotalDispenseTargetML;
        }
        PourTargetML = (TotalDispenseTargetML - BloomTargetML) / 4.0;
        CurrentPour = 0;
        CurrentPourTargetML = BloomTargetML;
        CurrentPourDispensedML = 0.0;

        // Timeout includes a full allowed fill time and drain time for every 45 mL fill,
        // the bloom soak, and the three rests between the four regular pours.
        unsigned int fillCount = (unsigned int)((TotalDispenseTargetML + SHOWERHEAD_CAPACITY_ML - 0.001) /
                                                 SHOWERHEAD_CAPACITY_ML);
        StateDuration = (unsigned long)fillCount *
                            (FILL_FLOW_TIMEOUT_MS + SHOWERHEAD_DRAIN_TIME_MS) +
                        BLOOM_SOAK_TIME_MS + (3 * PAUSE_BETWEEN_POURS_MS);
        StateStartTime = millis();

        StateFlags.DISPENSE_DispensingComplete = false;
        StateFlags.DISPENSE_TimeoutOccurred = false;
        StateFlags.GENERAL_Initialized = true;

        Serial.println("[DISPENSE] Starting volume-controlled bloom...");
        Serial.print("[DISPENSE] Total target: ");
        Serial.print(TotalDispenseTargetML);
        Serial.println(" mL");
        Serial.print("[DISPENSE] Bloom target: ");
        Serial.print(BloomTargetML);
        Serial.println(" mL");
        Serial.print("[DISPENSE] Each regular pour: ");
        Serial.print(PourTargetML);
        Serial.println(" mL");

        if (CurrentPourTargetML > 0.0) {
            StartShowerheadFill(CurrentPourTargetML);
        } else {
            CurrentDispensePhase = DISPENSE_BLOOM_SOAK;
            DispensePhaseStartTime = millis();
        }
        return;
    }

    if (millis() - StateStartTime > StateDuration) {
        StopDispenseWithError("[DISPENSE] TIMEOUT - Stopping dispense");
        return;
    }

    if (CurrentDispensePhase == DISPENSE_FILL_SHOWERHEAD) {
        unsigned long fillPulseCount = FlowPulseCount - FillStartPulseCount;
        float fillVolumeML = (float)fillPulseCount / FLOW_SENSOR_CALIBRATION;
        CurrentFlowRate = CalculateFlowRate();

        if (fillVolumeML >= CurrentFillTargetML) {
            // Stop adding water at the showerhead capacity; leave the valve open to drain.
            digitalWrite(PUMP_2_PIN, LOW);
            CurrentPourDispensedML += CurrentFillTargetML;
            CurrentDispensePhase = DISPENSE_DRAIN_SHOWERHEAD;
            DispensePhaseStartTime = millis();

            Serial.print("[DISPENSE] Fill complete: ");
            Serial.print(CurrentFillTargetML);
            Serial.println(" mL. Draining showerhead...");
            return;
        }

        if (millis() - DispensePhaseStartTime > FLOW_START_TIMEOUT_MS &&
            FlowPulseCount == FillStartPulseCount) {
            StopDispenseWithError("[DISPENSE] ERROR: No flow detected while filling showerhead");
            return;
        }

        if (millis() - DispensePhaseStartTime > FILL_FLOW_TIMEOUT_MS) {
            StopDispenseWithError("[DISPENSE] ERROR: Showerhead fill timed out");
            return;
        }
        return;
    }

    if (CurrentDispensePhase == DISPENSE_DRAIN_SHOWERHEAD) {
        if (millis() - DispensePhaseStartTime < SHOWERHEAD_DRAIN_TIME_MS) {
            return;
        }

        digitalWrite(SOLENOID_PIN, LOW);
        if (CurrentPourDispensedML + 0.01 < CurrentPourTargetML) {
            StartShowerheadFill(CurrentPourTargetML - CurrentPourDispensedML);
            return;
        }

        if (CurrentPour == 0) {
            CurrentDispensePhase = DISPENSE_BLOOM_SOAK;
            DispensePhaseStartTime = millis();
            Serial.println("[DISPENSE] Bloom complete. Starting bloom soak...");
            return;
        }

        if (CurrentPour >= 4) {
            CurrentDispensePhase = DISPENSE_COMPLETE;
        } else {
            CurrentDispensePhase = DISPENSE_PAUSE_BETWEEN_POURS;
            DispensePhaseStartTime = millis();
            Serial.println("[DISPENSE] Pour complete. Starting 30 second rest...");
        }
    }

    if (CurrentDispensePhase == DISPENSE_BLOOM_SOAK) {
        if (millis() - DispensePhaseStartTime < BLOOM_SOAK_TIME_MS) {
            return;
        }

        if (PourTargetML <= 0.0) {
            CurrentDispensePhase = DISPENSE_COMPLETE;
            return;
        }

        CurrentPour = 1;
        CurrentPourTargetML = PourTargetML;
        CurrentPourDispensedML = 0.0;
        StartShowerheadFill(CurrentPourTargetML);
        Serial.println("[DISPENSE] Starting regular pour 1 of 4");
        return;
    }

    if (CurrentDispensePhase == DISPENSE_PAUSE_BETWEEN_POURS) {
        if (millis() - DispensePhaseStartTime < PAUSE_BETWEEN_POURS_MS) {
            return;
        }

        CurrentPour++;
        CurrentPourTargetML = PourTargetML;
        CurrentPourDispensedML = 0.0;
        StartShowerheadFill(CurrentPourTargetML);
        Serial.print("[DISPENSE] Starting regular pour ");
        Serial.print(CurrentPour);
        Serial.println(" of 4");
        return;
    }

    if (CurrentDispensePhase == DISPENSE_COMPLETE) {
        digitalWrite(PUMP_2_PIN, LOW);
        digitalWrite(SOLENOID_PIN, LOW);
        detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN_2));

        StateFlags.DISPENSE_DispensingComplete = true;
        StateFlags.GENERAL_Initialized = false;
        // Back to IDLE (was GRIND) so a finished brew reports "IDLE" over
        // BLE - the app's active-brew screen specifically watches for
        // status flipping to IDLE right after DISPENSE to show the "Enjoy
        // your Coffee!" screen. Leaving this as GRIND made the app jump
        // back to the grinding screen the instant the real coffee finished.
        CurrentState = IDLE;
        StateStartTime = millis();

        Serial.println("[DISPENSE] ========== DISPENSE COMPLETE ==========");
        Serial.print("[DISPENSE] Target water dispensed: ");
        Serial.print(TotalDispenseTargetML);
        Serial.println(" mL");
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
    pinMode(HEATER_PIN, OUTPUT);
    digitalWrite(HEATER_PIN, LOW);
    BleSetup();
    HandleTARE();
}

void loop(){

    // BLE work deferred from callbacks - see the comment above
    // BleStatusUpdatePending for why this can't happen inside onWrite/etc.
    if (BleAdvertisingRestartPending) {
        BLEDevice::startAdvertising();
        BleAdvertisingRestartPending = false;
    }
    if (BleStatusUpdatePending) {
        BleSendStatusUpdate();
        BleStatusUpdatePending = false;
    }

    static unsigned long lastBleNotify = 0;
    if (pBleServer->getConnectedCount() > 0 && millis() - lastBleNotify > 2000) {
        BleSendStatusUpdate();
        lastBleNotify = millis();
    }

    switch(CurrentState){
        case IDLE:
            //HandleIDLE();
            break;
        case GRIND:
            HandleGRIND();
            break;
        case USER_PROMPT:
            //HandleUSER_PROMPT();
            break;
        case HEAT:
            //HandleHEAT();
            break;
        case DISPENSE:
            //HandleDISPENSE();
            break;
        case ERROR:
            //HandleERROR();
            break;
    }

}
