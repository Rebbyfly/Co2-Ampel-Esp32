#include <WiFi.h>
#include <PubSubClient.h>
#include <MHZ19.h>
#include <HardwareSerial.h>
#include <string.h>

const char* gSSID = "";
const char* gPasswort = "";
const char* gMQTT_Broker = "";

const int gPinLedGruen = 18;
const int gPinLedGelb = 0;
const int gPinLedRot = 4;

const int RX_PIN = 16;  // ESP32 empfängt
const int TX_PIN = 17;  // ESP32 sendet

WiFiClient espClient;
PubSubClient mqtt_client(espClient);
MHZ19 co2Sensor;

HardwareSerial mySerial(1);  // UART1

void setup_wifi() {
  WiFi.begin(gSSID, gPasswort);
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

void reconnect() {
  while(!mqtt_client.connected()) {
    Serial.print("Versuche MQTT-Verbindung…");
    String vClientId = "ESP32_CO2_" + WiFi.macAddress();
    if(mqtt_client.connect(vClientId.c_str())) {
      Serial.println(" verbunden!");
    } else {
      Serial.print(" fehlgeschlagen, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" => erneuter Versuch in 5s");
      zeigeFehler(gPinLedGelb);
      delay(5000);
    }
  }
}

void setup() {
  pinMode(gPinLedGruen, OUTPUT);
  pinMode(gPinLedGelb, OUTPUT);
  pinMode(gPinLedRot, OUTPUT);

  Serial.begin(115200); // Seriellen Monitor
  Serial.println("Starte CO2-Messung...");
  
  mySerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); // Sensor
  co2Sensor.begin(mySerial);  // Hier wird der Standard-Serial Port verwendet
  
  // Optional: Auto-Kalibrierung deaktivieren
  co2Sensor.autoCalibration(false);

  setup_wifi();
  mqtt_client.setServer(gMQTT_Broker, 1883);
}

void schalteAlleLedsAus() {
  digitalWrite(gPinLedGruen, LOW);
  digitalWrite(gPinLedGelb, LOW);
  digitalWrite(gPinLedRot, LOW);
}

void sprecheZugehoerigeLedBeiWertAn(int pPPM) {
  schalteAlleLedsAus();

  if(pPPM <= 800) {
    digitalWrite(gPinLedGruen, HIGH);
  } else if (pPPM <= 1400) {
    digitalWrite(gPinLedGelb, HIGH);
  } else {
    digitalWrite(gPinLedRot, HIGH);
  }
}

void zeigeFehler(int pExtraPin) {
  schalteAlleLedsAus();
  digitalWrite(gPinLedRot, HIGH);
  if(pExtraPin >= 0) {
    digitalWrite(pExtraPin, HIGH);
  }
}

bool sendeDatenAnBroker(int pPPM) {
  char vPayload[10];
  snprintf(vPayload, sizeof(vPayload), "%d", pPPM);  // wandelt int in char[]
  return mqtt_client.publish("co2/ppm", vPayload);
}

void loop() {
  if(WiFi.status() != WL_CONNECTED) {
    setup_wifi();
  }
  if(!mqtt_client.connected()) {
    reconnect();
  }
  mqtt_client.loop();

  int vAktuellerCo2Wert = co2Sensor.getCO2();  // Hole CO2-Wert

  if(vAktuellerCo2Wert > 0) {  // Gültiger Messwert
    Serial.print("CO2: ");
    Serial.print(vAktuellerCo2Wert);
    Serial.println(" ppm");
    sendeDatenAnBroker(vAktuellerCo2Wert);
    sprecheZugehoerigeLedBeiWertAn(vAktuellerCo2Wert);
  } else {
    Serial.println("Fehler beim Auslesen des Sensors.");
  }

  delay(2000); // 2 Sekunden warten
}
