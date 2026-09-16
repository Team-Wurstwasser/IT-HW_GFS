# ESP-NOW Roboterarm & EV3 Farbsortier-System

Dieses Projekt implementiert eine drahtlose Farberkennungs- und Sortiersteuerung für einen 4-Achs-Roboterarm (angetrieben durch SCServo/Feetech Smart Servos). Das System besteht aus zwei ESP32-Controllern, die über **ESP-NOW** (ein schnelles, verbindungsloses WLAN-Protokoll) miteinander kommunizieren.

---

## 🛠️ Systemarchitektur

Das Projekt ist als PlatformIO-Multi-Environment-Setup aufgebaut und in zwei eigenständige Firmware-Komponenten aufgeteilt, die über `build_src_filter` getrennt kompiliert werden:

### 1. Sender / Transmitter (`src/transmitter/`)
* **Hardware:** ESP32 + LEGO Mindstorms EV3 Farbsensor.
* **Funktion:** Initialisiert den EV3-Sensor über ein proprietäres serielles Protokoll (Handshake bei 2400 Baud, danach Wechsel auf 57600 Baud) und fordert aktiv den Farbmodus an. Anschließend wird zyklisch (alle 200 ms) der aktuelle Farbwert ausgelesen und per ESP-NOW an den Empfänger gesendet.

### 2. Empfänger / Receiver (`src/receiver/`)
* **Hardware:** ESP32 + Feetech STS/SMS Smart Servos (über Hardware-Serial bei 1 MBaud).
* **Funktion:** Empfängt die Farbsignale, glättet sie über einen Ringpuffer (Rauschfilterung) und berechnet mittels **inverser Kinematik (IK)** die Gelenkwinkel, um Objekte basierend auf ihrer Farbe präzise zu greifen und auf vordefinierten Koordinaten zu sortieren.

### Projektstruktur

```
src/
├── receiver/
│   ├── main.cpp
│   ├── config.h
│   ├── hardware/
│   │   ├── gripper.cpp / gripper.h
│   │   └── calibration.cpp / calibration.h
│   ├── math/
│   │   └── kinematics.cpp / kinematics.h
│   └── comm/
│       └── esp_now_handler.cpp / esp_now_handler.h
└── transmitter/
    ├── main.cpp
    ├── config.h
    ├── hardware/
    │   └── ev3_sensor.cpp / ev3_sensor.h
    └── comm/
        └── esp_now_handler.cpp / esp_now_handler.h
```

---

## 📐 Mathematische Kinematik & Servos

Der Roboterarm nutzt geometrische inverse Kinematik, um kartesische Koordinaten $(X, Y, Z)$ und den Endeffektor-Winkel ($\theta$) in Servo-Ticks (0–4095) umzurechnen. Zentrale Funktion ist `ctrlIK()`, die folgende Teilschritte kombiniert:

* **`calculateXYPlane`**: Berechnet die Basisdrehung (Achse 1, `angle_1`) und kompensiert den seitlichen Achsversatz (`LEN_H`), um die effektive horizontale Restlänge für die Gelenkkette zu ermitteln.
* **`calculateGripper`**: Berechnet den Vorhaltekeil des Endeffektors (Handgelenk-Ausrichtung, `angle_EoAT`) sowie die daraus resultierende Positionskorrektur, abhängig vom Zielwinkel `theta`.
* **`calculateJointChain`**: Löst über den Kosinussatz das Dreiecksproblem der Hauptachsen (Schulter `angle_2` und Ellenbogen `angle_3`) anhand der Armsegmente `LEN_C` und `LEN_D` und bestimmt den Korrekturwinkel für das Handgelenk (Achse 4).

Die berechneten Winkel werden in `ctrlMove()` in Servo-Steps (`angleGenOut`) umgerechnet und per `SyncWritePosEx` synchron an alle vier Hauptgelenke gesendet.

---

## 📋 Farbtabelle & Sortierlogik

Das System erkennt bis zu 8 Zustände (0–7). Je nach erkannter Farbe wird das Objekt über die Koordinatentabelle `dropLocations[]` zu einer anderen Zielposition gebracht:

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

Farbwerte außerhalb des gültigen Bereichs (0–7) werden von `dropBlock()` sicherheitshalber auf den Standardplatz (ID 0) abgebildet.

---

## 📡 Farberkennung & Signalfilterung

Da die ESP-NOW-Übertragung Ausreißer und Rauschen enthalten kann, wird die Farberkennung zweistufig stabilisiert:

1. **Empfangs-Callback (`OnDataRecv`)**: Nimmt eingehende Werte nur alle 100 ms entgegen (Entprellung) und schreibt sie in einen 20 Werte großen Ringpuffer.
2. **`getFilteredColor()`**: Ermittelt beim Abruf den im Puffer am häufigsten vorkommenden Farbwert (Modalwert) und liefert diesen als stabiles Ergebnis zurück.

Im Ablauf wartet der Arm nach dem Anfahren der Scan-Position 4 Sekunden, damit genügend Messwerte über ESP-NOW eintreffen können, bevor `getFilteredColor()` ausgewertet wird.

---

## ⚙️ Homing & Kalibrierung (`s2AutoConfig`)

Beim Start führt der Empfänger-ESP32 eine automatische Kalibrierung für **Achse 2** durch:

1. Die Achsen 1, 3, 4 und 5 fahren in eine definierte, sichere Ausgangsposition.
2. Achse 2 schaltet in den **Dauerrotationsmodus** (`setMode(2, 3)`) und fährt langsam gegen den mechanischen Anschlag.
3. Der ESP32 überwacht laufend die Servolast (`readServoLoad`). Überschreitet diese den Schwellenwert (`torqueThreshold = 300`), wird der Motor gestoppt.
4. Der Arm fährt ein definiertes Stück zurück frei (`reverseRange`), setzt diesen Punkt als neuen Nullpunkt (`setMiddle`) und wechselt zurück in den **Positionsmodus**.

Sowohl beim Homing als auch bei jeder regulären Positionierung wird bei Signalverlust eines Servos der letzte Befehl automatisch erneut gesendet, bis wieder eine gültige Rückmeldung vorliegt.

---

## 🤏 Greifersteuerung

`openGripper()` und `closeGripper()` fahren den Greifer-Servo (ID 5) zu festen Zielpositionen (1300 bzw. 2300) und überwachen dabei die Bewegung:

* Bewegt sich die Position über 150 ms hinweg um weniger als 8 Steps, wird das als mechanische Blockade bzw. als gegriffenes Objekt gewertet, die aktuelle Position wird fixiert und die Funktion beendet.
* Geht die Verbindung zum Servo verloren, wird der Zielbefehl automatisch erneut gesendet.

---

## 🔄 Programmablauf

### Empfänger (`receiver/main.cpp`, `loop()`)
1. **Scan-Position anfahren:** Der Arm bewegt sich über die Sicherheits-Überflugsposition (`PICKUP_HOVER`) zur Scan-Position (`PICKUP_SCAN`, `Z: 85.0`) direkt über dem Objekt.
2. **Warten & Filtern:** 4 Sekunden Wartezeit, damit ausreichend Farbwerte per ESP-NOW eintreffen; anschließend liefert `getFilteredColor()` den stabilsten Wert der letzten 20 Samples.
3. **Zupacken:** Der Arm senkt sich auf die Greifkoordinate (`PICKUP_GRAB`) ab, `closeGripper()` schließt den Greifer bis zur Blockade-Erkennung, danach hebt `PICKUP_LIFT` das Objekt an.
4. **Sortieren:** `dropBlock(colorID)` fährt anhand der erkannten Farb-ID die passende Koordinate aus `dropLocations[]` an und öffnet dort per `openGripper()` den Greifer.
5. Der Zyklus beginnt danach unmittelbar von vorn.

### Sender (`transmitter/main.cpp`, `loop()`)
1. `sensorRead()` liest den aktuellen Farbwert vom EV3-Sensor aus.
2. `sendColorData()` überträgt den Wert per ESP-NOW an den Empfänger.
3. 200 ms Pause, danach erneuter Zyklus.

---

## 🚀 Installation & Inbetriebnahme

### 1. Voraussetzungen
Das Projekt nutzt PlatformIO. Benötigte Abhängigkeiten (siehe `platformio.ini`):
* `SCServo` (Feetech, für die Empfänger-Firmware)
* ESP32-Arduino-Core (`WiFi`, `esp_now` – bereits im Framework enthalten)

### 2. MAC-Adresse anpassen
Vor dem Hochladen muss die MAC-Adresse des Empfänger-ESP32 ausgelesen und im **Sender-Code** (`transmitter/config.h`) eingetragen werden:
```cpp
const uint8_t receiverAddress[] = {0x28, 0x05, 0xa5, 0x4f, 0xcf, 0xe0};
```

### 3. Firmware bauen und hochladen
```bash
# Empfänger (Roboterarm-Steuerung)
pio run -e receiver -t upload

# Sender (Farbsensor-Modul)
pio run -e transmitter -t upload
```

---

## 🔌 Pinbelegung

### 📡 Sender (Farbsensor-Modul)
<img width="170" height="241" alt="pin_sensor_connection" src="https://github.com/user-attachments/assets/a197ee79-b29b-477e-a35c-ff98eb3c46e3" />

| Pin | Funktion |
|:---:|---|
| **26** (`EV3_RX_PIN`) | Verbunden mit dem TX-Pin des LEGO Mindstorms EV3 Farbsensors |
| **27** (`EV3_TX_PIN`) | Verbunden mit dem RX-Pin des LEGO Mindstorms EV3 Farbsensors |

### 🤖 Empfänger (Roboterarm-Steuerung)

| Pin | Funktion |
|:---:|---|
| **18** (`S_RXD`) | Empfangsleitung für das Feetech/SCServo Bussystem |
| **19** (`S_TXD`) | Sendeleitung für das Feetech/SCServo Bussystem |

> **Hinweis:** Da es sich um ein Halbduplex-Eindraht-Bussystem handelt, wird hier typischerweise eine entsprechende Treiberschaltung oder ein Signal-Inverter verwendet.