#include <SCServo.h> 

#define S_RXD 18
#define S_TXD 19
#define RS485_CTRL_PIN 2 

SMS_STS st; 

void setup() {
  Serial.begin(115200);
  
  pinMode(RS485_CTRL_PIN, OUTPUT);
  digitalWrite(RS485_CTRL_PIN, LOW);
  
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  
  delay(1000);
  Serial.println("System gestartet...");
}

void loop() {
  digitalWrite(RS485_CTRL_PIN, HIGH);
  delayMicroseconds(10);
  
  if(st.FeedBack(1) != -1) {
    digitalWrite(RS485_CTRL_PIN, LOW);
    int pos = st.ReadPos(-1);
    Serial.print("Position ID 1: ");
    Serial.println(pos);
  } else {
    digitalWrite(RS485_CTRL_PIN, LOW);
    Serial.println("Fehler: Kein Servo antwortet.");
  }
  
  delay(500);
}