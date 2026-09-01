/*
 * ═══════════════════════════════════════════════════════════════════
 *   AgriAI Lab — ESP32 BLE Firmware
 *   CognoSpace Experiential Learning
 *
 *   Sensors:
 *     - DHT11  → Temperature & Humidity  (pin 25)
 *     - Rain   → Digital (LOW = rain)    (pin 33)
 *     - Soil   → Digital (HIGH = dry)    (pin 32)
 *
 *   Motor Driver L298N:
 *     - IN1 → 13
 *     - IN3 → 26
 *     - ENA → 14
 *     - ENB → 27
 *
 *   BLE Device Name:
 *     AgriAILab-TESLA
 *
 *   BLE Service UUID:
 *     12345678-1234-1234-1234-123456789abc
 *
 *   Sensor Characteristic:
 *     12345678-1234-1234-1234-123456789012
 *
 *     Sends:
 *     T:xx.x,H:xx,R:x,S:x
 *
 *   Motor Characteristic:
 *     12345678-1234-1234-1234-123456789013
 *
 *     Receives:
 *     forward:200
 *     backward:200
 *     left:180
 *     right:180
 *     stop:0
 * ═══════════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <DHT.h>

// ═══════════════════════════════════════════════════════════════════
// PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════════

#define DHT_PIN   15
#define DHT_TYPE  DHT11

#define RAIN_PIN  33
#define SOIL_PIN  32

// L298N
#define IN1  13
#define IN3  26
#define ENA  14
#define ENB  27

// PWM
#define PWM_FREQ  5000
#define PWM_RES   8


// ═══════════════════════════════════════════════════════════════════
// BLE UUIDs
// ═══════════════════════════════════════════════════════════════════

#define SERVICE_UUID \
"12345678-1234-1234-1234-123456789abc"

#define SENSOR_UUID \
"12345678-1234-1234-1234-123456789012"

#define MOTOR_UUID \
"12345678-1234-1234-1234-123456789013"


// ═══════════════════════════════════════════════════════════════════
// GLOBAL VARIABLES
// ═══════════════════════════════════════════════════════════════════

DHT dht(DHT_PIN, DHT_TYPE);

BLEServer* pServer = nullptr;

BLECharacteristic* pSensorChr = nullptr;

BLECharacteristic* pMotorChr = nullptr;

bool deviceConnected = false;

unsigned long lastSend = 0;


// ═══════════════════════════════════════════════════════════════════
// MOTOR FUNCTIONS
// ═══════════════════════════════════════════════════════════════════

void stopMotors() {

  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN3, LOW);

  Serial.println("[MOTOR] STOP");
}


void moveForward(int spd) {

  spd = constrain(spd, 0, 255);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN3, LOW);

  ledcWrite(ENA, spd);
  ledcWrite(ENB, spd);

  Serial.printf("[MOTOR] FORWARD speed=%d\n", spd);
}


void moveBackward(int spd) {

  spd = constrain(spd, 0, 255);

  digitalWrite(IN1, LOW);
  digitalWrite(IN3, HIGH);

  ledcWrite(ENA, spd);
  ledcWrite(ENB, spd);

  Serial.printf("[MOTOR] BACKWARD speed=%d\n", spd);
}


void turnLeft(int spd) {

  spd = constrain(spd, 0, 255);

  digitalWrite(IN1, LOW);
  digitalWrite(IN3, LOW);

  ledcWrite(ENA, spd);
  ledcWrite(ENB, spd);

  Serial.printf("[MOTOR] TURN LEFT speed=%d\n", spd);
}


void turnRight(int spd) {

  spd = constrain(spd, 0, 255);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN3, HIGH);

  ledcWrite(ENA, spd);
  ledcWrite(ENB, spd);

  Serial.printf("[MOTOR] TURN RIGHT speed=%d\n", spd);
}


// ═══════════════════════════════════════════════════════════════════
// BLE SERVER CALLBACKS
// ═══════════════════════════════════════════════════════════════════

class ServerCB : public BLEServerCallbacks {

  void onConnect(BLEServer* s) override {

    deviceConnected = true;

    Serial.println("[BLE] Client connected");
  }


  void onDisconnect(BLEServer* s) override {

    deviceConnected = false;

    stopMotors();

    Serial.println("[BLE] Client disconnected");

    delay(500);

    s->startAdvertising();

    Serial.println("[BLE] Advertising restarted");
  }
};


// ═══════════════════════════════════════════════════════════════════
// MOTOR BLE CALLBACK
// ═══════════════════════════════════════════════════════════════════

class MotorCB : public BLECharacteristicCallbacks {

  void onWrite(BLECharacteristic* chr) override {

    String val = chr->getValue().c_str();

    val.trim();

    Serial.print("[BLE] Motor command: ");
    Serial.println(val);


    int colonIdx = val.indexOf(':');


    String cmd;

    int spd = 0;


    if (colonIdx >= 0) {

      cmd = val.substring(0, colonIdx);

      spd = val.substring(colonIdx + 1).toInt();

    } else {

      cmd = val;

    }


    spd = constrain(spd, 0, 255);

    cmd.toLowerCase();


    if (cmd == "forward") {

      moveForward(spd);

    }

    else if (cmd == "backward") {

      moveBackward(spd);

    }

    else if (cmd == "left") {

      turnLeft(spd);

    }

    else if (cmd == "right") {

      turnRight(spd);

    }

    else if (cmd == "stop") {

      stopMotors();

    }

    else {

      Serial.println("[BLE] Unknown motor command");

      stopMotors();

    }
  }
};


// ═══════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════

void setup() {

  Serial.begin(115200);

  delay(500);

  Serial.println();
  Serial.println("=================================");
  Serial.println("      AgriAI Lab ESP32");
  Serial.println("=================================");


  // ═══════════════════════════════════════════════════════════════
  // SENSORS
  // ═══════════════════════════════════════════════════════════════

  dht.begin();

  pinMode(RAIN_PIN, INPUT);

  pinMode(SOIL_PIN, INPUT);


  // ═══════════════════════════════════════════════════════════════
  // MOTOR
  // ═══════════════════════════════════════════════════════════════

  pinMode(IN1, OUTPUT);

  pinMode(IN3, OUTPUT);


  // ESP32 Arduino Core 3.x PWM API
  ledcAttach(ENA, PWM_FREQ, PWM_RES);

  ledcAttach(ENB, PWM_FREQ, PWM_RES);


  stopMotors();


  // ═══════════════════════════════════════════════════════════════
  // BLE
  // ═══════════════════════════════════════════════════════════════

  BLEDevice::init("AgriAILab-TESLA");


  pServer = BLEDevice::createServer();

  pServer->setCallbacks(new ServerCB());


  BLEService* pService =
      pServer->createService(SERVICE_UUID);


  // ═══════════════════════════════════════════════════════════════
  // SENSOR CHARACTERISTIC
  // ═══════════════════════════════════════════════════════════════

  pSensorChr =
      pService->createCharacteristic(

        SENSOR_UUID,

        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY

      );


  pSensorChr->addDescriptor(
      new BLE2902()
  );


  // ═══════════════════════════════════════════════════════════════
  // MOTOR CHARACTERISTIC
  // ═══════════════════════════════════════════════════════════════

  pMotorChr =
      pService->createCharacteristic(

        MOTOR_UUID,

        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_WRITE_NR

      );


  pMotorChr->setCallbacks(
      new MotorCB()
  );


  // ═══════════════════════════════════════════════════════════════
  // START BLE
  // ═══════════════════════════════════════════════════════════════

  pService->start();


  BLEAdvertising* pAdv =
      BLEDevice::getAdvertising();


  pAdv->addServiceUUID(SERVICE_UUID);

  pAdv->setScanResponse(true);

  pAdv->setMinPreferred(0x06);


  BLEDevice::startAdvertising();


  Serial.println();
  Serial.println("[BLE] Device Name: AgriAILab-TESLA");
  Serial.println("[BLE] Advertising started");
  Serial.println("[BLE] Waiting for connection...");
  Serial.println();
}


// ═══════════════════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════════════════

void loop() {

  // Do absolutely nothing if ESP32 is not connected
  if (!deviceConnected) {

    return;
  }


  unsigned long now = millis();


  // Send sensor data every 2 seconds

  if (now - lastSend < 2000) {

    return;
  }


  lastSend = now;


  // ═══════════════════════════════════════════════════════════════
  // READ DHT11
  // ═══════════════════════════════════════════════════════════════

  float temp = dht.readTemperature();

  float hum = dht.readHumidity();


  // If DHT11 fails, do NOT send fake data

  if (isnan(temp) || isnan(hum)) {

    Serial.println(
      "[DHT] Read failed - no sensor data sent"
    );

    return;
  }


  // ═══════════════════════════════════════════════════════════════
  // READ RAIN
  // LOW = RAIN
  // ═══════════════════════════════════════════════════════════════

  int rain =
      digitalRead(RAIN_PIN) == LOW ? 1 : 0;


  // ═══════════════════════════════════════════════════════════════
  // READ SOIL
  // HIGH = DRY
  // ═══════════════════════════════════════════════════════════════

  int soil =
      digitalRead(SOIL_PIN) == HIGH ? 1 : 0;


  // ═══════════════════════════════════════════════════════════════
  // CREATE SENSOR PACKET
  // ═══════════════════════════════════════════════════════════════

  char buf[48];


  snprintf(

    buf,

    sizeof(buf),

    "T:%.1f,H:%d,R:%d,S:%d",

    temp,

    (int)hum,

    rain,

    soil

  );


  // ═══════════════════════════════════════════════════════════════
  // SEND BLE NOTIFICATION
  // ═══════════════════════════════════════════════════════════════

  pSensorChr->setValue(buf);

  pSensorChr->notify();


  // ═══════════════════════════════════════════════════════════════
  // SERIAL MONITOR
  // ═══════════════════════════════════════════════════════════════

  Serial.printf(

    "[SENSOR] %s  | Rain: %s | Soil: %s\n",

    buf,

    rain ? "YES" : "NO",

    soil ? "DRY" : "MOIST"

  );
}