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
```

## 🔌 3. Pinbelegung

Die Hardware-Komponenten werden an die jeweiligen Pins der beiden ESP32-Controller wie folgt angeschlossen:

### 📡 Sender (Farbsensor-Modul)
<img width="170" height="241" alt="pin_sensor_connection" src="https://github.com/user-attachments/assets/a197ee79-b29b-477e-a35c-ff98eb3c46e3" />

* **Pin 26 (EV3_RX_PIN):** Verbunden mit dem TX-Pin des LEGO Mindstorms EV3 Farbsensors.
* **Pin 27 (EV3_TX_PIN):** Verbunden mit dem RX-Pin des LEGO Mindstorms EV3 Farbsensors.

### 🤖 Empfänger (Roboterarm-Steuerung)
* **Pin 18 (S_RXD):** Empfangsleitung für das Feetech/SCServo Bussystem (RXD).
* **Pin 19 (S_TXD):** Sendeleitung für das Feetech/SCServo Bussystem (TXD).
* *Hinweis: Da es sich um ein Halbduplex-Eindraht-Bussystem handelt, wird hier typischerweise eine entsprechende Treiberschaltung oder ein Signal-Inverter verwendet.*
---

## 🔄 4. Programmablauf (Loop)

Der automatische Hauptablauf (`loop()`) des Roboterarms folgt einer festen zyklischen Logik:

1. **Scan-Position anfahren:** Der Roboterarm bewegt sich zuerst auf eine sichere Warteposition direkt über dem Objekt (`X: 0.0, Y: 330.0, Z: 200.0, θ: 130.0`).
2. **Sensor absenken:** Der Arm senkt sich ab (`Z: 85.0`), um den EV3-Farbsensor nah genug an das Objekt heranzuführen. Hier verweilt er für 4 Sekunden, damit genügend Messwerte einfließen können.
3. **Filterung & Erkennung:** Der ESP32 liest die via ESP-NOW empfangenen Farb-IDs aus einem Ringpuffer aus. Die Funktion `holeGefilterteFarbe()` ermittelt den statistisch häufigsten Wert der letzten 20 Stichproben, um Signalrauschen zu eliminieren.
4. **Zupacken:** Der Arm wechselt auf die Greifkoordinate (`Z: 70.0`, leicht versetzt auf `Y: 351.0`). Die Funktion `closeGripper()` schließt den Greifer, bis ein mechanischer Widerstand (Blockade-Erkennung über Positionsabweichung) registriert wird.
5. **Anheben & Sortieren:** Das Objekt wird angehoben (`Z: 200.0`). Basierend auf der gefilterten `farbeID` wird über eine `switch-case`-Abfrage (`gebeSteinAb()`) die spezifische Zielbox-Koordinate angefahren.
6. **Abladen & Reset:** Am Zielort öffnet sich der Greifer (`openGripper()`). Der Arm verbleibt dort für 1,2 Sekunden, fährt dann zurück in die Ausgangsposition und wartet 2 Sekunden, bevor der nächste Sortierzyklus beginnt.

<br><br><br>
diese README wurde KI generiert
