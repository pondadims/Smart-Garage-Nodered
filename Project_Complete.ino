//@alfondadim
//@mhdferdialwan
//sok tanyakeun kalo bingung

#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include <MFRC522.h>

#define TRIGGER_PIN 4
#define ECHO_PIN 5
#define SERVO_PIN 13
#define IR_SENSOR_PIN 14
#define DISTANCE_THRESHOLD 5
#define RFID_SS_PIN 21
#define RFID_RST_PIN 22
#define INTERVAL 1000  // Interval pembacaan sensor dalam milidetik (ms)

Servo myservo; // servo object
int pos = 0; // Variabel untuk menyimpan posisi servo
int delayTime = 15; // Waktu delay untuk gerakan servo

unsigned long lastDetectionTime = 0;

MFRC522 mfrc522(RFID_SS_PIN, RFID_RST_PIN);

const int MAX_CARDS = 6;  
String registeredCards[MAX_CARDS] = {
  "371165245",
  "514093134661620",
  "45483138249110128",
};

bool CarisInside = false;
const char *ssid = "loveciboy";
const char *password = "tomdanoscar0403";
const char *mqtt_server = "test.mosquitto.org"; // MQTT broker
const char* mqtt_topic1 = "/Node-RED-Co";
const char* mqtt_topic2 = "/Node-RED-Ci";
const char* mqtt_topic3 = "/Node-RED-Do";
const char* mqtt_topic4 = "/Node-RED-Dc";
const char* mqtt_topic5 = "/ThinkIOT/Servo-nodered";
const char* mqtt_topic6 = "/Node-RED-User";



WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;


void setup()
{
    Serial.begin(115200);
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(IR_SENSOR_PIN, INPUT);
    myservo.attach(SERVO_PIN);

    SPI.begin();
    mfrc522.PCD_Init();
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
}



void setup_wifi()
{
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  
  if (strcmp(topic, mqtt_topic5) == 0) {
    char command = payload[0];
    
    if (command == 'O') {
      openServo();
    } else if (command == 'C') {
      closeServo();
    }
  }
}
void reconnect()
{
    while (!client.connected())
    {
        Serial.print("Attempting MQTT connection...");
        if (client.connect("ESPClient"))
        {
            Serial.println("connected");
            client.subscribe("/ThinkIOT/Servo-nodered");
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  handleRFID();
  ultrasonicCondition();
  infrared();
  delay(100);
}

void handleRFID() {
  // Periksa apakah kartu RFID ditempatkan di depan pembaca
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    Serial.println("Kartu RFID terdeteksi!");

    // Baca serial number kartu dan tampilkan di Serial Monitor
    String cardSerial = getCardSerial();
    Serial.println("Serial Number Kartu: " + cardSerial);
  

    // Periksa apakah kartu terdaftar
    if (isCardRegistered(cardSerial)) {
      //Serial.println("Kartu terdaftar. Membuka servo...");
        openServo();
        delay(2000);
        client.publish(mqtt_topic6, cardSerial.c_str());
        delay(3000);
        closeServo();
    } else {
      Serial.println("Kartu tidak terdaftar.");
    }
  }
}

String getCardSerial() {
  String cardSerial = "";
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    cardSerial += String(mfrc522.uid.uidByte[i]);
  }
  return cardSerial;
}

bool isCardRegistered(String cardSerial) {
  for (int i = 0; i < MAX_CARDS; i++) {
    if (registeredCards[i] == cardSerial) {
      return true; // Kartu terdaftar
    }
  }
  return false; // Kartu tidak terdaftar
}

bool ultrasonicCondition()
{
    long duration, distance;

    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);

    duration = pulseIn(ECHO_PIN, HIGH);
    distance = duration * 0.034 / 2;

    if (distance < DISTANCE_THRESHOLD)
    {
        openServo();
        delay(5000);
        closeServo();
        return true;
    }
    return false;
}

bool cardetected() {
  // Logika deteksi objek berdasarkan pembacaan sensor infrared
  return digitalRead(IR_SENSOR_PIN) == HIGH;  // Sesuaikan dengan logika sensor Anda
}

void infrared()
{
    if (cardetected())
    {
        if (!CarisInside)
        {
            Serial.println("Car is Outside");
            client.publish(mqtt_topic1, "Car is Outside");
            CarisInside = true;
        }
    }
    else
    {
        if (CarisInside)
        {
            Serial.println("Car is Inside");
            client.publish(mqtt_topic2, "Car is Inside");
            CarisInside = false;
        }
    }

    delay(INTERVAL);
}

void openServo() 
{
    // Buka servo secara pelan
  for (pos = 0; pos <= 180; pos += 1) {
    myservo.write(pos);
    delay(delayTime); // Tambahkan delay yang kecil untuk memberikan efek pelan
  }
  Serial.println("Door is Open");
  client.publish(mqtt_topic3, "Door is Open");
}

void closeServo()
{
for (pos = 180; pos >= 0; pos -= 1) {
    myservo.write(pos);
    delay(delayTime); // Tambahkan delay yang kecil untuk memberikan efek pelan
  }
  Serial.println("Door is Close");
  client.publish(mqtt_topic4, "Door is Close");
}



   
