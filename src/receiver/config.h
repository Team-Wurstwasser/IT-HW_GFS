#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Pin-Belegung für die serielle Kommunikation mit den Servos
const uint8_t S_RXD = 18;
const uint8_t S_TXD = 19;

// Geometrische Längenmaße des Roboterarms
const float LEN_A = 115.432, LEN_B = 41.0513, LEN_C = 168.8579, LEN_D = 127.9234, LEN_E = 108.5357, LEN_F = 7.7076, LEN_G = 90.0, LEN_H = -13.75;

// Indizes für das InfoBuffer-Array zur Speicherung berechneter Gelenkwinkel und Längen
const uint8_t angle_1 = 0, angle_2 = 1, angle_3 = 2, angle_4 = 3, angle_IKE = 4, angle_EoAT  = 5, len_a = 6, len_b = 7, len_totalXY = 8;

// Servo-IDs im Bussystem
const uint8_t STID_count = 4;      // Anzahl der Hauptgelenke
const uint8_t gripperID = 5;       // Servo-ID für den Greifer

// Konfigurationswerte für die mechanische Blockade-Erkennung
const int torqueThreshold = 300;   // Last-Schwellenwert für die Erkennung des Anschlags
const int reverseRange = 4900;     // Distanz für das Freifahren nach Anschlag
const int positionTolerance = 25;  // Toleranz in Schritten, wann ein Servo sein Ziel erreicht hat

// Zuordnung der Farb-IDs zu Klartext-Namen
const String tableColor[8] = {
  "No color", "Black", "Blue", "Green", "Yellow", "Red", "White", "Brown"
};

// Struktur für kartesische Koordinaten und den Handgelenk-Winkel 
struct Coordinate { float x, y, z, theta; };

// Fest definierte Bahnpunkte für den Arbeitsablauf
const Coordinate PICKUP_HOVER = {0.0, 330.0, 200.0, 130.0}; // Sicherheits-Überflugsposition
const Coordinate PICKUP_SCAN  = {0.0, 330.0,  85.0, 130.0}; // Position zum Scannen der Farbe
const Coordinate PICKUP_GRAB  = {0.0, 351.0,  70.0, 250.0}; // Absenken und Stein greifen
const Coordinate PICKUP_LIFT  = {0.0, 351.0, 200.0, 250.0}; // Stein nach oben wegholen

// Ziel-Abladeplätze sortiert nach Farb-ID (0 bis 7)
const Coordinate dropLocations[8] = {
  { 250.0, -70.0, 95.0, 250.0 },  // 0: No color (Default-Platz)
  { 250.0, -70.0, 95.0, 250.0 },  // 1: Black
  { -70.0, -315.0, 95.0, 250.0 }, // 2: Blue
  {  25.0, -315.0, 95.0, 250.0 }, // 3: Green
  { -70.0, -250.0, 95.0, 250.0 }, // 4: Yellow
  {  25.0, -250.0, 95.0, 250.0 }, // 5: Red
  { 250.0,   25.0, 95.0, 250.0 }, // 6: White
  { 250.0, -70.0, 95.0, 250.0 }   // 7: Brown
};

// Globale Hilfsfunktion zur Umrechnung von Grad in Servo-Steps
inline int16_t angleGenOut(float angleInput) {
    return (int16_t)(angleInput * 4096.0 / 360.0 + 0.5);
}

#endif