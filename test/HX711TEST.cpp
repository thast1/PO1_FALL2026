/* Testing File for HX711 & Load Cell Components. 
Specific to ECEN 403 Pour Over 1 Capstone Project
------ AUTOMATED POUR OVER COFFEE MACHINE -------*/

#include <Arduino.h> // Arduino Framework
#include "HX711.h" // HX711 ADC Signal Amplifier for Load Cells

#define DOUT_1          17
#define CLK_1           20
#define DOUT_2          10
#define CLK_2           11

HX711 LoadCellBeans_1; // 4541 Load Cell
HX711 LoadCellBeans_2; // 4541 Load Cell
float CalibrationFactor_1      = 0.0;
float CalibrationFactor_2      = 0.0;

void setup() {
    Serial.begin(57600);
    LoadCellBeans_1.begin(DOUT_1, CLK_1);
    LoadCellBeans_2.begin(DOUT_2, CLK_2);

}

void loop(){
    if (LoadCellBeans_1.is_ready() && LoadCellBeans_2.is_ready()) {
        long reading_1 = LoadCellBeans_1.read() + CalibrationFactor_1;
        long reading_2 = LoadCellBeans_2.read() + CalibrationFactor_2;
        long avg_reading = reading_1 + reading_2;
        Serial.print("HX711_1 Readings: ");
        Serial.println(reading_1);
        Serial.print("HX711_2 Readings: ");
        Serial.println(reading_2);
        Serial.print("Combined Load Readings: ");
        Serial.println(avg_reading);
    } 
    else {
        Serial.println("HX711 not found.");
    }

    delay(5000);

}