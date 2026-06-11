# ESP-NOW Roboterarm & EV3 Farbsortier-System

Dieses Projekt implementiert eine drahtlose Farberkennungs- und Sortiersteuerung für einen 4-Achs-Roboterarm (angetrieben durch SCServo/Feetech Smart Servos). Das System besteht aus zwei ESP32-Controllern, die über **ESP-NOW** (ein schnelles, verbindungsloses WLAN-Protokoll) miteinander kommunizieren.

---

## 🛠️ Systemarchitektur

Das Projekt ist in zwei eigenständige Komponenten aufgeteilt:

### 1. Sender (Farbsensor-Modul)
* **Hardware:** ESP32 + LEGO Mindstorms EV3 Farbsensor.
* **Funktion:** Initialisiert den EV3-Sensor über ein proprietäres serielles Protokoll (erst 2400 Baud zur Synchronisation, dann Wechsel auf 57600 Baud). Es liest kontinuierlich ID-Werte aus (Farb-Modus) und sendet diese zyklisch via ESP-NOW an den Roboterarm.

### 2. Empfänger (Roboterarm-Steuerung)
* **Hardware:** ESP32 + Feetech STS/SMS Smart Servos (über Hardware-Serial bei 1 MBaud).
* **Funktion:** Empfängt die Farbsignale, glättet sie über einen Ringpuffer (Rauschfilterung) und berechnet mittels **Inverser Kinematik (IK)** die Gelenkwinkel, um Objekte basierend auf ihrer Farbe präzise zu greifen und auf vordefinierten Koordinaten zu sortieren.

---

## 📐 Mathematische Kinematik & Servos

Der Roboterarm nutzt geometrische Inverse Kinematik, um kartesische Koordinaten $(X, Y, Z)$ und den Endeffektor-Winkel ($\theta$) in Servo-Ticks (0–4095) umzurechnen.

* **`wigglePlaneIK`**: Berechnet die Drehung der Basis (Achse 1) und kompensiert den Achsversatz (`LEN_H`).
* **`EoAT_IK` (End of Arm Tooling)**: Berechnet den Vorhaltekeil für das Handgelenk.
* **`simpleLinkageIK`**: Löst das trigonometrische Problem des Hauptarms (Achse 2 & 3) basierend auf den Armsegmenten (`LEN_C` und `LEN_D`).

---

## 📋 Farbtabelle & Sortierlogik

Das System erkennt bis zu 8 Zustände (0–7). Je nach erkannter Farbe wird das Objekt zu einer anderen Zielkoordinate gebracht:

| ID | Farbe | Aktion / Zielort |
|:---:|---|---|
| **0** | No color | Standard-Ablageplatz (`X: 250, Y: -70`) |
| **1** | Black | Standard-Ablageplatz |
| **2** | Blue | Koordinate: `-70.0, -315.0, 95.0` |
| **3** | Green | Koordinate: `25.0, -315.0, 95.0` |
| **4** | Yellow | Koordinate: `-70.0, -250.0, 95.0` |
| **5** | Red | Koordinate: `25.0, -250.0, 95.0` |
| **6** | White | Koordinate: `250.0, 25.0, 95.0` |
| **7** | Brown | Standard-Ablageplatz |

---

## ⚙️ Homing & Kalibrierung (`s2AutoConfig`)

Beim Start führt der Empfänger-ESP32 eine automatische Drehmoment-Kalibrierung für **Achse 2** durch:
1. Die Achsen 1, 3, 4 und 5 fahren in eine sichere Ausgangsposition.
2. Achse 2 schaltet in den **Dauerrotationsmodus (Wheel Mode)** und fährt langsam gegen den mechanischen Anschlag.
3. Der ESP32 überwacht die Stromlast (`readServoLoad`). Überschreitet diese den Schwellenwert (`torqueThreshold = 300`), stoppt der Motor.
4. Der Arm fährt ein Stück zurück, setzt diesen Punkt als Nullpunkt (`setMiddle`) und wechselt zurück in den **Positionsmodus**.

---

## 🚀 Installations- und Ablaufanleitung

### 1. Voraussetzungen (Libraries)
Stelle sicher, dass folgende Bibliotheken in der Arduino IDE installiert sind:
* `SCServo` (von Feetech / Active 1)
* `WiFi` (ESP32 Core)
* `esp_now` (ESP32 Core)

### 2. MAC-Adresse anpassen
Bevor du den Code hochlädst, musst du die MAC-Adresse des Empfängers auslesen und im **Sender-Code** eintragen:
```cpp
uint8_t receiverAddress[] = {0x28, 0x05, 0xa5, 0x4f, 0xcf, 0xe0};
