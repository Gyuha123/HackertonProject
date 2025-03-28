#include <WiFi.h>
#include <Arduino.h>

const int dioder = 32;
const int diodeg = 33;
const int diodeb = 34;
const int CLK = 26;
const int DT = 27;
const int buzzer = 13;

const char* ssid = "WIFI";
const char* password = "sion0526";

WiFiServer server(80);

unsigned long previousMillis = 0;
const long interval = 50;
bool blinkState = false;
bool blinkMode = false;

int colorIndex = 0;
const int colors[][3] = {
  {HIGH, LOW, LOW},
  {LOW, HIGH, LOW},
  {LOW, LOW, HIGH},
  {HIGH, HIGH, LOW},
  {LOW, HIGH, HIGH},
  {HIGH, LOW, HIGH}
};

unsigned long buzzerPreviousMillis = 0;
const long buzzerInterval = 100;
bool buzzerState = false;
bool buzzerBlinkMode = false;
bool buzzerOnMode = false;

void setColor(int r, int g, int b) {
  digitalWrite(dioder, r);
  digitalWrite(diodeg, g);
  digitalWrite(diodeb, b);
}

void setup() {
  Serial.begin(115200);
  pinMode(dioder, OUTPUT);
  pinMode(diodeg, OUTPUT);
  pinMode(diodeb, OUTPUT);
  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);
  pinMode(buzzer, OUTPUT);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    tone(buzzer, 1000);
    delay(500);
    Serial.print(".");
    noTone(buzzer);
    delay(500);
  }

  Serial.println("\nWiFi connected.");
  Serial.print("IP address: ");
  delay(100);
  Serial.println(WiFi.localIP());

  tone(buzzer, 1000);
  delay(100);
  noTone(buzzer);
  delay(100);
  tone(buzzer, 1000);
  delay(100);
  noTone(buzzer);

  server.begin();
}

void loop() {
  WiFiClient client = server.available();

  if (client) {
    String currentLine = "";
    while (client.connected()) {  
      if (client.available()) {
        char c = client.read();
        if (c == '\n') {
          if (currentLine.length() == 0) {
            client.println("HTTP/1.1 200 OK\nContent-type:text/html\n\nGWAKGYUHAPOOP\n");
            break;
          } else {
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
        
        if (currentLine.endsWith("GET /H")) {
          pinMode(dioder, OUTPUT);
          pinMode(diodeg, OUTPUT);
          pinMode(diodeb, OUTPUT); 
          blinkMode = false;
          buzzerBlinkMode = true;
          buzzerOnMode = false;
          buzzerPreviousMillis = 0;
          buzzerState = false;
        } else if (currentLine.endsWith("GET /L")) {
          pinMode(dioder, INPUT);
          pinMode(diodeg, INPUT);
          pinMode(diodeb, INPUT);
          buzzerBlinkMode = false;
          buzzerOnMode = false;
        } else if (currentLine.endsWith("GET /HB")) {
          pinMode(dioder, OUTPUT);
          pinMode(diodeg, OUTPUT);
          pinMode(diodeb, OUTPUT); 
          blinkMode = true;
          buzzerBlinkMode = false;
          buzzerOnMode = true;
          tone(buzzer, 2000);
        }
      }
    }
    client.stop();
  } 

  if (blinkMode) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      blinkState = !blinkState;
      setColor(blinkState ? colors[colorIndex][0] : LOW, blinkState ? colors[colorIndex][1] : LOW, blinkState ? colors[colorIndex][2] : LOW);
    }
  } else {
    setColor(colors[colorIndex][0], colors[colorIndex][1], colors[colorIndex][2]);
  }

  static int prevCLKState = digitalRead(CLK);   
  int currentCLKState = digitalRead(CLK);

  if (prevCLKState != currentCLKState && currentCLKState == HIGH) {
    colorIndex = (digitalRead(DT) == LOW) ? (colorIndex + 1) % 6 : (colorIndex - 1 + 6) % 6;
    delay(50);
  }
  prevCLKState = currentCLKState;

  if (buzzerBlinkMode) {
    unsigned long currentMillis = millis();
    if (currentMillis - buzzerPreviousMillis >= buzzerInterval) {
      buzzerPreviousMillis = currentMillis;
      buzzerState = !buzzerState;
      if (buzzerState) {
        tone(buzzer, 2000);
      } else {
        noTone(buzzer);
      }
    }
  } else if (!buzzerOnMode) {
    noTone(buzzer);
  }
}
