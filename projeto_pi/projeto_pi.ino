#include <MPU6050_tockn.h>
#include <Wire.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <HardwareSerial.h> 
#include <TinyGPS++.h>
#include <ArduinoOTA.h>

HardwareSerial gpsSerial(1); 
TinyGPSPlus gps; 

// Configuração do WiFi
const char* ssid = "Luis_F";
const char* password = "08100810";

// Configuração do Firebase
#define FIREBASE_HOST "https://projetointegrador-af3e1-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "VWpFLt7QvSNIasCA0b36ZovaXHCo1VGfUayJG7kh"

FirebaseData firebaseData;

MPU6050 mpu6050(Wire);
static float accY_ant = 0.0; 
static float accZ_ant = 0.0;  

void setup() {
  // Conexão com o WiFi
  Serial.begin(9600); 
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17); // inicializa a comunicação serial com o GPS, usando os pinos 16 (RX) e 17 (TX)
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando ao WiFi...");
  }
  Serial.println("Conexão WiFi estabelecida!");
  Serial.print("Endereço IP do ESP32: ");
  Serial.println(WiFi.localIP());
  
  // Inicialização do Firebase
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);

  ArduinoOTA.begin();
  Serial.println("OTA habilitado!");
  ArduinoOTA.setPassword("12345678");
}


void loop() {
  ArduinoOTA.handle();

  mpu6050.update();
  float accY = mpu6050.getAccY();
  float accZ = mpu6050.getAccZ();
 
  if(abs(accY - accY_ant) > 0.9 || abs(accY - accY_ant) < -0.9 || abs(accZ - accZ_ant) > 0.9 || abs(accZ - accZ_ant) < -0.9){
    Serial.println("Queda detectada!");
    Firebase.setString(firebaseData, "/status", "Queda detectada!");
  } else {
    Serial.println("Sem queda.");
    Firebase.setString(firebaseData, "/status", "Sem queda");
  }

  Serial.println("=======================================================");
  Serial.print("accX : ");Serial.print(mpu6050.getAccX());
  Serial.print("\taccY : ");Serial.print(mpu6050.getAccY());
  Serial.print("\taccZ : ");Serial.println(mpu6050.getAccZ());

  Serial.print("gyroX : ");Serial.print(mpu6050.getGyroX());
  Serial.print("\tgyroY : ");Serial.print(mpu6050.getGyroY());
  Serial.print("\tgyroZ : ");Serial.println(mpu6050.getGyroZ());

  Serial.print("accAngleX : ");Serial.print(mpu6050.getAccAngleX());
  Serial.print("\taccAngleY : ");Serial.println(mpu6050.getAccAngleY());

  Serial.print("gyroAngleX : ");Serial.print(mpu6050.getGyroAngleX());
  Serial.print("\tgyroAngleY : ");Serial.print(mpu6050.getGyroAngleY());
  Serial.print("\tgyroAngleZ : ");Serial.println(mpu6050.getGyroAngleZ());

  Serial.print("angleX : ");Serial.print(mpu6050.getAngleX());
  Serial.print("\tangleY : ");Serial.print(mpu6050.getAngleY());
  Serial.print("\tangleZ : ");Serial.println(mpu6050.getAngleZ());
  Serial.println("=======================================================\n");

  accY_ant = accY;
  accZ_ant = accZ;

  unsigned long gpsStartTime = millis(); // marca o tempo de início da leitura do GPS
  unsigned long interval = 50; // define o intervalo de tempo para cada leitura (50 ms)

  while (millis() - gpsStartTime < 500) { // lê o GPS por meio segundo
    unsigned long currentTime = millis();

    while (gpsSerial.available() > 0 && millis() - currentTime < interval) { // enquanto houver dados disponíveis na comunicação serial do GPS e o intervalo de tempo não tiver sido atingido
      if (gps.encode(gpsSerial.read())) { // processa os dados recebidos pelo GPS
        if (gps.location.isValid()) { // se a localização atual do GPS é válida
          float latitude , longitude;
          Serial.print("Latitude: ");
          Serial.print(gps.location.lat(), 6); 
          Serial.print(", Longitude: ");
          Serial.println(gps.location.lng(), 6); 
          latitude = gps.location.lat();
          longitude = gps.location.lng();
          Firebase.setFloat(firebaseData, "gps/latitude", latitude);
          Firebase.setFloat(firebaseData, "gps/longitude", longitude);
        }
      }
    }

    mpu6050.update(); // atualiza os dados do MPU6050
  }
}