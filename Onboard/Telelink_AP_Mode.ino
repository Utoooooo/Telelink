#include <WebServer.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <HardwareSerial.h>
#include <MSP.h>

#include <VL53L1X.h>
#include <Wire.h>

// the IP of the machine to which you send msgs - this should be the correct IP in most cases (see note in python code)
#define CONSOLE_IP "192.168.1.2"
#define CONSOLE_PORT 4210
const char* ssid = "TELELINK";
const char* password = "12345678";
WiFiUDP Udp;
IPAddress local_ip(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
WebServer server(80);

HardwareSerial Telemetry(1);
MSP msp;

VL53L1X dist;

/// @brief Telemtry structure, edit when needed 
struct Telem  {
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    uint16_t tof;
  };
Telem telelink;

void setup()
{
  // Setup UDP Server
  Serial.begin(115200);
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  server.begin();

  // Setup MSP Communication
  Telemetry.begin(115200, SERIAL_8N1, 15, 14);
  msp.begin(Telemetry);

  //// TOF Sensor Setup
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2Csensor.setTimeout(500);
  dist.setTimeout(500);
  
  if (!dist.init()){
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  dist.setDistanceMode(VL53L1X::Short);
  dist.setMeasurementTimingBudget(20000);
  dist.startContinuous(20);
}

void loop()
{
  recv_telem();
}



/// @brief Request telemetry data from FC's MSP port
void recv_telem()
{
  msp_raw_imu_t imu;
  msp_attitude_t attd;
    if (msp.request(MSP_RAW_IMU, &imu, sizeof(imu))) {
      int16_t *acc     = imu.acc;
      int16_t *gyro    = imu.gyro;
      int16_t *mag     = imu.mag;
    }
    if (msp.request(MSP_ATTITUDE, &attd, sizeof(attd))) {
      int16_t roll     = attd.roll;
      int16_t pitch    = attd.pitch;
      int16_t yaw     = attd.yaw;
    }

    // if (msp.request(MSP_ANALOG, &anlg, sizeof(anlg))) {
    //   uint8_t  vbat      = anlg.vbat;       // 0...255
    //   uint16_t mAhDrawn  = anlg.mAhDrawn;   // milliamp hours drawn from battery
    //   uint16_t rssi      = anlg.rssi;       // 0..1023
    //   int16_t  amperage  = anlg.amperage;   // send amperage in 0.01 A steps, range is -320A to 320A 
    // }
  telelink = {imu.acc[0], imu.acc[1], imu.acc[2], attd.roll, attd.pitch, attd.yaw, dist.read()};
  // telelink = {imu.acc[0], imu.acc[1], imu.acc[2], attd.roll, attd.pitch, attd.yaw};
  printToSerial(telelink);
  broadCast(telelink);
  
}

/// @brief Print telemetry data to serial port, debugging/demonstration use only
/// @param Telem 
void printToSerial(Telem &Telem) {
  Serial.print("Acc_x: ");
  Serial.println(Telem.acc_x);
  Serial.print("Acc_y: ");
  Serial.println(Telem.acc_y);
  Serial.print("Acc_z: ");
  Serial.println(Telem.acc_z);
  Serial.print("Roll: ");
  Serial.println(Telem.roll);
  Serial.print("Pitch: ");
  Serial.println(Telem.pitch);
  Serial.print("Yaw: ");
  Serial.println(Telem.yaw);
  Serial.print("ToF: ");
  Serial.println(Telem.tof);
}

/// @brief broadcast telemetry data in WIFI
/// @param data 
void broadCast(Telem &data) {
  Udp.beginPacket(CONSOLE_IP, CONSOLE_PORT);

  Udp.write((const uint8_t *)&data, sizeof(Telem));

  Udp.endPacket();
}
