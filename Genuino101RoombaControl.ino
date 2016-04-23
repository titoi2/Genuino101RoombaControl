#include <BLEAttribute.h>
#include <BLECentral.h>
#include <BLECharacteristic.h>
#include <BLECommon.h>
#include <BLEDescriptor.h>
#include <BLEPeripheral.h>
#include <BLEService.h>
#include <BLETypedCharacteristic.h>
#include <BLETypedCharacteristics.h>
#include <BLEUuid.h>
#include <CurieBLE.h>
#include <AquesTalk.h>
#include <Wire.h>  // I2Cライブラリ
#include <SoftwareSerial.h>

AquesTalk atp;  // 音声合成のインスタンス

BLEPeripheral blePeripheral;       // BLE Peripheral Device (the board you're programming)
BLEService r2cService("44E00000-5E8D-448F-9D33-3EF6E598AD7A"); // BLE Roomba2Coco Service

// BLE  Characteristic"
BLECharCharacteristic r2cChar("44E00001-5E8D-448F-9D33-3EF6E598AD7A",
                              BLERead | BLEWrite);

// remote clients will be able to get notifications if this characteristic changes
// the characteristic is 2 bytes long as the first field needs to be "Flags" as per BLE specifications
// https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.heart_rate_measurement.xml

int oldHeartRate = 0;  // last heart rate reading from analog input
long previousMillis = 0;  // last time the heart rate was checked, in ms

static const int COMMAND_BUFFER_SIZE = 127;
static char commandBuffer[COMMAND_BUFFER_SIZE + 1];
static int commandBufferIndex;

static  SoftwareSerial roombaSerial(10, 12);
static int16_t  leftWheelVelocity;  // ルンバ左送信値
static int16_t  rightWheelVelocity;  // ルンバ右送信値

void setup() {
  Wire.begin();

  Serial.begin(115200);    // initialize serial communication
  pinMode(13, OUTPUT);   // initialize the LED on pin 13 to indicate when a central is connected


  roombaSerial.begin(115200);


  /* Set a local name for the BLE device
     This name will appear in advertising packets
     and can be used by remote devices to identify this BLE device
     The name can be changed but maybe be truncated based on space left in advertisement packet */
  blePeripheral.setLocalName("Roomba2Coco");
  blePeripheral.setAdvertisedServiceUuid(r2cService.uuid());  // add the service UUID
  blePeripheral.addAttribute(r2cService);   // Add the BLE Heart Rate service
  blePeripheral.addAttribute(r2cChar); // add the Heart Rate Measurement characteristic

  r2cChar.setEventHandler(BLEWritten, switchCharacteristicWritten);

  /* Now activate the BLE device.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection */
  blePeripheral.begin();
  Serial.println("Bluetooth device active, waiting for connections...");

  commandBufferIndex = 0;

  leftWheelVelocity = 0;  // ルンバ左送信値
  rightWheelVelocity = 0;  // ルンバ右送信値
}


void loop() {
  BLECentral central = blePeripheral.central();

}

void switchCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {
  // central wrote new value to characteristic, update LED
  Serial.print("Characteristic event, written: ");

  unsigned char v = r2cChar.value();
  Serial.println(v);

  commandBuffer[commandBufferIndex] = v;
  commandBufferIndex++;
  if (v == 0x0D) {
    commandBuffer[commandBufferIndex] = 0;
    commandAnalyze(commandBufferIndex);
    commandBufferIndex = 0;
    return;
  }
  if (commandBufferIndex >= COMMAND_BUFFER_SIZE) {
    commandBufferIndex = 0;
  }
}

void roombaSend()
{
  Serial.print("leftWheelVelocity:");
  Serial.println(leftWheelVelocity);
  Serial.print("rightWheelVelocity:");
  Serial.println(rightWheelVelocity);



  static uint8_t command[7];

  command[0] = 128; // start
  command[1] = 132; // full
  command[2] = 145; // direct drive
  command[3] = (rightWheelVelocity & 0xFF00) >> 8;  //MSB
  command[4] = rightWheelVelocity & 0xFF;           //LSB
  command[5] = (leftWheelVelocity  & 0xFF00) >> 8;  //MSB
  command[6] = leftWheelVelocity  & 0xFF;           //LSB
  roombaSerial.write(command, 7);

}

void commandAnalyze(int length) {
  if (length < 2) {
    return;
  }


  Serial.print("Command:");
  Serial.println(commandBuffer[0]);

  switch (commandBuffer[0]) {
    case 'l': {
        Serial.println("Roomba left");

        // ルンバ左車輪制御
        if (commandBuffer[1] != ':') {
          return;
        }

        leftWheelVelocity = atoi(commandBuffer + 2);  // ルンバ左送信値
        roombaSend();
      }
      break;

    case 'r': {

        Serial.println("Roomba right");
        // ルンバ右車輪制御
        if (commandBuffer[1] != ':') {
          return;
        }

        rightWheelVelocity = atoi(commandBuffer + 2);  // ルンバ左送信値
        roombaSend();
      }
      break;

    case 's': {
        Serial.println("Roomba stop");

        // ルンバ停止
        leftWheelVelocity = 0;  // ルンバ左送信値
        rightWheelVelocity = 0;  // ルンバ右送信値
        roombaSend();
      }
      break;

    case 't': {
        if (commandBuffer[1] != ':') {
          return;
        }
        Serial.print("SAY:");
        Serial.println(commandBuffer + 2);
        atp.Write(commandBuffer + 2);
      }
      break;
  }
}

