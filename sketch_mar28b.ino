#include <BasicLinearAlgebra.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// WiFi and MQTT Setup
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Coordinate conversion constants
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

// WiFi Configuration
const char *ssid = "Dialog 4G 010";
const char *password = "jukikali99";

// Hardware Serial for GPS
HardwareSerial gpsSerial(1); // UART1 (RX=16, TX=17)

// Sensor Fusion Global Variables
double fusedLat = 0.0;
double fusedLon = 0.0;
SemaphoreHandle_t xFusionMutex;

// MPU6050 and Kalman Filter variables
MPU6050 mpu;
bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
Quaternion q;
VectorFloat gravity;

// Kalman Filter Matrices
const double EARTH_RADIUS = 6378137.0;
const double EARTH_GRAVITY_MS2 = 9.80665;
unsigned long lastTime = 0;

// FreeRTOS Queues
QueueHandle_t imuQueue;
QueueHandle_t gpsQueue;

// Reference point for ENU conversion
double reffusedLat = 0.0;
double reffusedLon = 0.0;
bool refSet = false;
int fixCount = 0;

BLA::Matrix<4, 4> Q = {0.1, 0,    0,    0,
                       0,    0.1, 0,    0,
                       0,    0,    0.5, 0,
                       0,    0,    0,    0.5};

BLA::Matrix<2, 2> R = {10.0, 0,
                       0,   10.0};

BLA::Matrix<4, 4> Pk_1 = {1, 0, 0, 0,
                         0, 1, 0, 0,
                         0, 0, 1, 0,
                         0, 0, 0, 1};

BLA::Matrix<4, 1> xk_1 = {0, 0, 0, 0}; // State vector [x, y, vx, vy]

BLA::Matrix<2, 4> H = {1, 0, 0, 0,
                      0, 1, 0, 0};

BLA::Matrix<4, 4> I = {1, 0, 0, 0,
                      0, 1, 0, 0,
                      0, 0, 1, 0,
                      0, 0, 0, 1};

void setupWifi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void setupMqtt() {
  mqttClient.setServer("broker.mqtt.cool", 1883);
}

void connectToBroker() {
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT...");
    if (mqttClient.connect("ESP32-Kalman")) {
      Serial.println("MQTT Connected!");
    } else {
      Serial.print("Failed: ");
      Serial.println(mqttClient.state());
      delay(5000);
    }
  }
}

BLA::Matrix<2, 1> convertGPSToENU(double lat, double lon) {
    double latRad = lat * DEG_TO_RAD;
    double lonRad = lon * DEG_TO_RAD;
    double refLatRad = reffusedLat * DEG_TO_RAD;
    double refLonRad = reffusedLon * DEG_TO_RAD;

    double E = EARTH_RADIUS * (lonRad - refLonRad) * cos(refLatRad);
    double N = EARTH_RADIUS * (latRad - refLatRad);

    return {E, N};
}

void convertENUToGPS(double E, double N, double& lat, double& lon) {
    double refLatRad = reffusedLat * DEG_TO_RAD;
    double refLonRad = reffusedLon * DEG_TO_RAD;

    lon = (refLonRad + (E / (EARTH_RADIUS * cos(refLatRad)))) * RAD_TO_DEG;
    lat = (refLatRad + (N / EARTH_RADIUS)) * RAD_TO_DEG;
}

void TaskReadIMU(void *pvParameters) {
    uint8_t fifoBuffer[64];
    VectorInt16 aa, aaReal, aaWorld;

    for (;;) {
        if (dmpReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpConvertToWorldFrame(&aaWorld, &aaReal, &q);

            BLA::Matrix<2, 1> imuData = {
                aaWorld.x * EARTH_GRAVITY_MS2,
                aaWorld.y * EARTH_GRAVITY_MS2
            };
            xQueueSend(imuQueue, &imuData, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void TaskReadGPS(void *pvParameters) {
    char nmeaSentence[128];
    int nmeaIndex = 0;

    for (;;) {
        while (gpsSerial.available()) {
            char c = gpsSerial.read();
            if (c == '\n' && nmeaIndex > 0) {
                nmeaSentence[nmeaIndex] = '\0';
                if (strncmp(nmeaSentence, "$GNRMC", 6) == 0) {
                    char *fields[12];
                    char *token = strtok(nmeaSentence, ",");
                    int fieldIndex = 0;
                    
                    while (token && fieldIndex < 12) {
                        fields[fieldIndex++] = token;
                        token = strtok(NULL, ",");
                    }

                    if (fieldIndex >= 10 && fields[2][0] == 'A') {
                        double lat = atof(fields[3]) / 100.0;
                        lat = floor(lat) + (lat - floor(lat)) * 100.0 / 60.0;
                        if (fields[4][0] == 'S') lat *= -1;

                        double lon = atof(fields[5]) / 100.0;
                        lon = floor(lon) + (lon - floor(lon)) * 100.0 / 60.0;
                        if (fields[6][0] == 'W') lon *= -1;

                        // Set reference after 5 valid fixes
                        if (!refSet) {
                            fixCount++;
                            if (fixCount >= 5) {
                                reffusedLat = lat;
                                reffusedLon = lon;
                                refSet = true;
                                Serial.printf("Reference set: %.6f, %.6f\n", lat, lon);
                            }
                        } else {
                            BLA::Matrix<2, 1> gpsData = convertGPSToENU(lat, lon);
                            xQueueSend(gpsQueue, &gpsData, 0);
                        }
                    }
                }
                nmeaIndex = 0;
            } else if (nmeaIndex < 127) {
                nmeaSentence[nmeaIndex++] = c;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void TaskSensorFusion(void *pvParameters) {
    BLA::Matrix<2, 1> uk, zk;
    unsigned long lastGPSTime = 0;

    for (;;) {
        float delta_t = (millis() - lastTime) / 1000.0;
        lastTime = millis();

        // Update dynamic matrices
        BLA::Matrix<4, 4> A = {1, 0, delta_t, 0,
                               0, 1, 0, delta_t,
                               0, 0, 1, 0,
                               0, 0, 0, 1};

        BLA::Matrix<4, 2> B = {0.5 * delta_t * delta_t, 0,
                              0, 0.5 * delta_t * delta_t,
                              delta_t, 0,
                              0, delta_t};

        // Check queues
        bool hasIMU = xQueueReceive(imuQueue, &uk, 0);
        bool hasGPS = xQueueReceive(gpsQueue, &zk, 0);

        if (hasIMU) {
            // Prediction step
            xk_1 = A * xk_1 + B * uk;
            Pk_1 = A * Pk_1 * ~A + Q;
        }

        if (hasGPS) {
            // Update step
            lastGPSTime = millis();
            BLA::Matrix<2, 1> yk = zk - H * xk_1;
            BLA::Matrix<2, 2> S = H * Pk_1 * ~H + R;
            
            if (Determinant(S) != 0) {
                BLA::Matrix<4, 2> K = Pk_1 * ~H * Inverse(S);
                xk_1 = xk_1 + K * yk;
                Pk_1 = (I - K * H) * Pk_1;

                // Convert and store fused coordinates
                double currentLat, currentLon;
                convertENUToGPS(xk_1(0), xk_1(1), currentLat, currentLon);
                
                if (xSemaphoreTake(xFusionMutex, portMAX_DELAY)) {
                    fusedLat = currentLat;
                    fusedLon = currentLon;
                    xSemaphoreGive(xFusionMutex);
                }
            }
        } else if (millis() - lastGPSTime > 5000) {
            // Increase process noise if GPS is stale
            Q(0,0) = 2.0; Q(1,1) = 2.0;
            Q(2,2) = 5.0; Q(3,3) = 5.0;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {
    Serial.begin(115200);
    setupWifi();
    setupMqtt();

    // IMU Initialization
    Wire.begin(21, 22);  // Replace with your actual pin numbers
    while (!Serial);
    Serial.println("Scanning I2C bus...");

  for (byte i = 1; i < 127; ++i) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at: 0x");
      Serial.println(i, HEX);
    }
  }
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    
    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.println("DMP Init Failed!");
        while(1);
    }

    // GPS Initialization
    gpsSerial.begin(9600, SERIAL_8N1, 16, 17);

    // Create queues and mutex
    imuQueue = xQueueCreate(5, sizeof(BLA::Matrix<2, 1>));
    gpsQueue = xQueueCreate(5, sizeof(BLA::Matrix<2, 1>));
    xFusionMutex = xSemaphoreCreateMutex();

    // Create tasks
    xTaskCreate(TaskReadIMU, "IMU", 8192, NULL, 2, NULL);
    xTaskCreate(TaskReadGPS, "GPS", 8192, NULL, 2, NULL);
    xTaskCreate(TaskSensorFusion, "Fusion", 16384, NULL, 3, NULL);
}

void loop() {
    if (!mqttClient.connected()) connectToBroker();
    mqttClient.loop();
    sendValues();
    delay(10);
}

void sendValues(){
  static unsigned long lastSend = 0;
    if (millis() - lastSend > 1000) {
        StaticJsonDocument<200> doc;
        double lat, lon;
        
        if (xSemaphoreTake(xFusionMutex, portMAX_DELAY)) {
            lat = fusedLat;
            lon = fusedLon;
            xSemaphoreGive(xFusionMutex);
        }

        doc["latValue"] = lat;
        doc["lonValue"] = lon;
        
        String output;
        serializeJson(doc, output);
        mqttClient.publish("MAP", output.c_str());
        lastSend = millis();
    }
}