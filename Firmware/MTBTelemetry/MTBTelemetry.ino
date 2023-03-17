//Define the pin functions -- stolen from OLA code
//Depends on hardware version. This can be found as a marking on the PCB.
//x04 was the SparkX 'black' version.
//v10 was the first red version.
#define HARDWARE_VERSION_MAJOR 1
#define HARDWARE_VERSION_MINOR 0

#if(HARDWARE_VERSION_MAJOR == 0 && HARDWARE_VERSION_MINOR == 4)
const byte PIN_MICROSD_CHIP_SELECT = 10;
const byte PIN_IMU_POWER = 22;
#elif(HARDWARE_VERSION_MAJOR == 1 && HARDWARE_VERSION_MINOR == 0)
const byte PIN_MICROSD_CHIP_SELECT = 23;
const byte PIN_IMU_POWER = 27;
const byte PIN_PWR_LED = 29;
const byte PIN_VREG_ENABLE = 25;
const byte PIN_VIN_MONITOR = 34; // VIN/3 (1M/2M - will require a correction factor)
#endif

const byte PIN_POWER_LOSS = 3;
//const byte PIN_LOGIC_DEBUG = 11; // Useful for debugging issues like the slippery mux bug
const byte PIN_MICROSD_POWER = 15;
const byte PIN_QWIIC_POWER = 18;
const byte PIN_STAT_LED = 19;
const byte PIN_IMU_INT = 37;
const byte PIN_IMU_CHIP_SELECT = 44;
const byte PIN_STOP_LOGGING = 32;
const byte BREAKOUT_PIN_32 = 32;
const byte BREAKOUT_PIN_TX = 12;
const byte BREAKOUT_PIN_RX = 13;
const byte BREAKOUT_PIN_11 = 11;
const byte PIN_TRIGGER = 11;
const byte PIN_QWIIC_SCL = 8;
const byte PIN_QWIIC_SDA = 9;

const byte PIN_SPI_SCK = 5;
const byte PIN_SPI_CIPO = 6;
const byte PIN_SPI_COPI = 7;

unsigned long qwiicPowerOnTime = 0; //Used to track when we last powered on the Qwiic bus

#include <Wire.h> //I2C Library
TwoWire qwiic(PIN_QWIIC_SDA,PIN_QWIIC_SCL);
#include "qwiic.h" //Qwiic helper functions

#include <Arduino.h>
#include <SPI.h> //SPI Library
//#include <SdFat.h> //SD Card Library

#include "SparkFun_u-blox_GNSS_Arduino_Library.h" //http://librarymanager/All#SparkFun_u-blox_GNSS
#include "SparkFun_ISM330DHCX.h" // Click here to get the library: http://librarymanager/All#SparkFun_6DoF_ISM330DHCX
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

#include "my_dtostrf.h" //Custom dtostrf function
#include "support.h"


//include Apollo RTC
#include "RTC.h"
Apollo3RTC myRTC;

//Sensor Addresses
#define IMU_ISM330DHCX 0x6B
#define IMU_ISM330DHCX_ALT 0x6A
#define LIDAR_VL53L1X 0x29
#define UBX_GNSS 0x42
#define IMU_ICM20948_SPI 0x00


/*
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*/


class BTSensor {
  private:
    uint8_t address;
    uint8_t dataOutputs = 0;
    bool logStaleData = false;
    String helperText = "";
    bool running = false; //Is the sensor running?
    bool lastLog = false; //Did the sensor log last time it was asked to? 

    virtual bool checkData() {
      //Check if new data is available
      return false;
    }

    virtual void readData() {
      //Read new data
    }

    virtual String logData() {
      //Log data
      return "";
    }

  public:
    BTSensor(uint8_t address) {
      //Sensor constructor
      this->address = address;
      Serial.print("Sensor object created. Address: 0x");
      Serial.println(address, HEX);
    }
    ~BTSensor() {
      //Sensor destructor 
    }

    virtual bool begin() {
      //Sensor begin
      Serial.println("Attempting to begin generic sensor object. Nothing to do.");
      return false;
    }

    void set_running (bool running) {
      this->running = running;
    }

    bool is_running() {
      return this->running;
    }

    void setDataOutputs(uint8_t dataOutputs) {
      this->dataOutputs = dataOutputs;
    }

    void setHelperText(String helperText) {
      this->helperText = helperText;
    }

    void setAddress(uint8_t address) {
      this->address = address;
    }

    uint8_t getAddress() {
      return this->address;
    }

    void setLogStaleData(bool logStale) {
      this->logStaleData = logStale;
    }

    String log() {
      bool dataReady = this->checkData();
      String logline = "";
      if( dataReady ) {
        //New data is available. Read new data.
        this->readData();
      } else {
        //Data is stale
      }

      if( dataReady || this->logStaleData) {
        logline += this->logData();
      } else {
        //Add empty data to logline
        logline += this->noLog();

      }

      //Record if we logged data
      this->lastLog = (dataReady || this->logStaleData);

      return logline;
    }

    String noLog() {
      String logline = "";
      for (int i = 0; i < this->dataOutputs; i++) {
        logline += ",";
      }
      return logline;
    }

    bool didLog() {
      return this->lastLog;
    }

    String getHelperText() {
      return this->helperText;
    }

    String getAddressHelperText() {
      String logline = "";
      for (int i = 0; i < this->dataOutputs; i++) {
        char addressString[10];
        sprintf(addressString, "0x%X", this->address);
        logline += "," + String(addressString);
      }
      return logline;
    }

    void setLogStale(bool logStale) {
      this->logStaleData = logStale;
    }
};

class BT_ISM330DHCX : public BTSensor {
  private:
    SparkFun_ISM330DHCX *sensorPtr;
    sfe_ism_data_t *accelData;
    sfe_ism_data_t *gyroData;

    bool checkData() override {
      //Check if new data is available
      bool new_data = (this->sensorPtr)->checkStatus();
      return new_data;
    }

    void readData() override {
      //Read new data
      sfe_ism_data_t* accelData = new sfe_ism_data_t;
      sfe_ism_data_t* gyroData = new sfe_ism_data_t;

      bool gotAccell = (this->sensorPtr)->getAccel(accelData);
      bool gotGyro = (this->sensorPtr)->getGyro(gyroData);

      delete this->accelData;
      delete this->gyroData;

      this->accelData = accelData;
      this->gyroData = gyroData;
    }

    String logData() override {
      String logline = "";
      char buffer1[10];
      char buffer2[10];
      char buffer3[10];
      char buffer4[10];
      char buffer5[10];
      char buffer6[10];

      sfe_ism_data_t accelData = *(this->accelData);
      sfe_ism_data_t gyroData = *(this->gyroData);

      //Add new data to logline
      olaftoa(accelData.xData, buffer1, 3, sizeof(buffer1) / sizeof(char));
      olaftoa(accelData.yData, buffer2, 3, sizeof(buffer2) / sizeof(char));
      olaftoa(accelData.zData, buffer3, 3, sizeof(buffer3) / sizeof(char));
      olaftoa(gyroData.xData, buffer4, 3, sizeof(buffer4) / sizeof(char));
      olaftoa(gyroData.yData, buffer5, 3, sizeof(buffer5) / sizeof(char));
      olaftoa(gyroData.zData, buffer6, 3, sizeof(buffer6) / sizeof(char));
      logline += "," + String(buffer1) 
        + "," + String(buffer2) 
        + "," + String(buffer3) 
        + "," + String(buffer4) 
        + "," + String(buffer5) 
        + "," + String(buffer6);
      return logline;
    }

  public:
    BT_ISM330DHCX(uint8_t address) : BTSensor(address) {  
      //ISM330DHCX constructor
      this->setDataOutputs(6);
      this->setHelperText(",AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ");
      Serial.println("ISM330DHCX sensor constructed.");
    }
    ~BT_ISM330DHCX() {
      //ISM330DHCX destructor
    }

    bool begin() override {
      //ISM330DHCX begin
      Serial.print("ISM330DHCX begin: 0x");
      Serial.println(this->getAddress(), HEX);
      SparkFun_ISM330DHCX *myISM = new SparkFun_ISM330DHCX;

      int tries = 0;

      while( !(myISM->begin(qwiic, this->getAddress())) ){
        Serial.println("Did not begin. Tying Again");
        delay(1000);
        if (tries > 5) {
          Serial.println("Failed to begin ISM330DHCX");
          return false;
        } else {
          tries++;
        }
      }

      // Reset the device to default settings. This if helpful is you're doing multiple
      // uploads testing different settings. 
      Serial.print("Resetting device.");
      myISM->deviceReset();

      // Wait for it to finish reseting
      while( !(myISM->getDeviceReset()) ){ 
        delay(10);
        Serial.print(".");
      } 
      Serial.println(" Done.");

      Serial.println("Applying settings.");
      delay(100);
      
      myISM->setDeviceConfig();
      myISM->setBlockDataUpdate();
      
      // Set the output data rate and precision of the accelerometer
      myISM->setAccelDataRate(ISM_XL_ODR_104Hz);
      myISM->setAccelFullScale(ISM_4g); 

      // Set the output data rate and precision of the gyroscope
      myISM->setGyroDataRate(ISM_GY_ODR_104Hz);
      myISM->setGyroFullScale(ISM_500dps); 

      // Turn on the accelerometer's filter and apply settings. 
      myISM->setAccelFilterLP2();
      myISM->setAccelSlopeFilter(ISM_LP_ODR_DIV_100);

      // Turn on the gyroscope's filter and apply settings. 
      myISM->setGyroFilterLP1();
      myISM->setGyroLP1Bandwidth(ISM_MEDIUM);

      //Wait for first data ready
      Serial.print("Waiting for data.");
      while(!(myISM->checkStatus())) {
        delay(100);
        Serial.print(".");
      }
      Serial.println(" Done!");

      this->sensorPtr = myISM;

      //TODO
      this->set_running(true);
      return true;
    }
};

class BT_VL53L1X : public BTSensor {
  private:
    SFEVL53L1X *sensorPtr = NULL;
    uint16_t rangeData = 0;
    uint16_t rangeStatus = 0;

    bool checkData() override {
      //Check if new data is available
      return (this->sensorPtr)->checkForDataReady();
    }

    void readData() override {
      //Read new data
      this->rangeData = (this->sensorPtr)->getDistance();
      this->rangeStatus = (this->sensorPtr)->getRangeStatus();
      (this->sensorPtr)->startOneshotRanging();

    }

    String logData() override {
      String logline = "";
      //Add new data to logline
      logline += "," + String(this->rangeData);
      if (this->rangeStatus > 0) {
        logline += ",1";
      } else {
        logline += ",0";
      }
      return logline;
    }

  public:
    BT_VL53L1X(uint8_t address) : BTSensor(address) {
      //VL53L1X constructor
      this->sensorPtr = new SFEVL53L1X(qwiic);
      this->setLogStaleData(false);
      this->setDataOutputs(2);
      this->setHelperText(",Range,RangeStatus");
      Serial.println("VL53L1X sensor constructed.");
    }
    ~BT_VL53L1X() {
      //VL53L1X destructor   
    }

    bool begin() override {
      //VL53L1X begin
      Serial.print("VL53L1X begin: 0x");
      Serial.println(this->getAddress(), HEX);
      bool begin_sucess = (this->sensorPtr)->begin() == 0;
      if (!begin_sucess) {
        Serial.println("Begin failed.");
      } else {
        //Begin ranging
        (this->sensorPtr)->startOneshotRanging();
      }
      this->set_running(begin_sucess);
      return(begin_sucess);
    }

};

/*
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*/


typedef struct {
  bool logSerial;
  unsigned long serialBaudRate;
  bool logSD;
  bool logRelativeTime;
  bool logFrequency;
  bool logTemperature;
  unsigned long minLogInterval;
} TrackerSettings;

struct SensorNode {
  unsigned long logInterval;
  uint8_t address;
  BTSensor *sensor;
  unsigned long lastLogTime;
  SensorNode *next;
};

SensorNode *sensorsHead = NULL;
SensorNode *sensorsTail = NULL;

TrackerSettings globalSettings;

unsigned long getInterval(float frequency) {
  return (1000 * (1.0 / frequency)); //Convert frequency to interval in milliseconds
}

void addSensor(SensorNode *node) {
  if (sensorsHead == NULL) {
    sensorsHead = node;
    sensorsTail = node;
  } else {
    sensorsTail->next = node;
    sensorsTail = node;
  }
}

void sdLogln(String *logline) {
  if (globalSettings.logSD) {
    //TODO: Log to SD card
  }
}

/*
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*/

static unsigned long lastLogTime;

void setup() {
  


  //TODO: Read settings from SD card
  globalSettings.logSerial = true;
  globalSettings.serialBaudRate = 115200;
  globalSettings.logSD = true;
  globalSettings.logRelativeTime = true;
  globalSettings.logFrequency = false;
  globalSettings.logTemperature = false;
  globalSettings.minLogInterval = getInterval(150.0);

  //Delay 2 seconds to allow sensors to power up
  delay(2000);

  //Turn on I2C
  Wire.begin();

  //Turn on SPI
  SPI.begin();

  //Begin qwiic
  beginQwiic();


  //If logging to serial, begin serial and wait for serial monitor
  if (globalSettings.logSerial) {
    Serial.begin(globalSettings.serialBaudRate);
    while (!Serial) {
      delay(10); // wait for serial port to connect. Needed for native USB port only
    }
  }

  //Detect and initialize sensors
  //TODO: Autodetect connected sensors
  addSensor(new SensorNode {
    getInterval(150.0), //150 Hz
    IMU_ISM330DHCX,
    new BT_ISM330DHCX(IMU_ISM330DHCX),
    millis(),
    NULL
  });
  addSensor(new SensorNode {
    getInterval(150.0), //150 Hz
    IMU_ISM330DHCX_ALT,
    new BT_ISM330DHCX(IMU_ISM330DHCX_ALT),
    millis(),
    NULL
  });
  addSensor(new SensorNode {
    getInterval(50.0), //50 Hz
    LIDAR_VL53L1X,
    new BT_VL53L1X(LIDAR_VL53L1X),
    millis(),
    NULL
  });
  //TODO: Add GNSS
  //TODO: Add ICM20948

  //begin sensors:
  SensorNode *node = sensorsHead;
  while(node != NULL) {
    if (node->sensor->begin()) {
      if (globalSettings.logSerial) {
        Serial.print("Sensor at address 0x");
        Serial.print(node->address, HEX);
        Serial.println(" initialized.");
      }
    } else {
      if (globalSettings.logSerial) {
        Serial.print("Sensor at address 0x");
        Serial.print(node->address, HEX);
        Serial.println(" failed to initialize.");

        //remove this sensor from the list
        if (node == sensorsHead) {
          sensorsHead = node->next;
        } else {
          SensorNode *prevNode = sensorsHead;
          while (prevNode->next != node) {
            prevNode = prevNode->next;
          }
          prevNode->next = node->next;
        }

      }
    }
    node = node->next;
  }

  Serial.println("Done initializing sensors.");

  //Print header:
  node = sensorsHead;
  String header = "\n\n\rBiketracker";
  //First line: sensor addresses
  while(node != NULL) {
    header += node->sensor->getAddressHelperText();
    node = node->next;
  }
  //Second line: data names
  node = sensorsHead;
  header += "\n\rTime_Millis";
  while(node != NULL) {
    header += node->sensor->getHelperText();
    node = node->next;
  }

  if (globalSettings.logSerial) {
    Serial.println(header);
  }
  sdLogln(&header);

  //Set up timer for logging
  lastLogTime = millis();

}


void loop() {
  unsigned long currentTime = millis();

  //check if enough time has elapsed between logs
  if(lastLogTime + globalSettings.minLogInterval <= currentTime) {
    lastLogTime = currentTime;

    //loop through sensors and log data
    SensorNode *node = sensorsHead;
    String logline = "\r";
    logline += String(currentTime);
    while(node != NULL) {

      //check if enough time has elapsed since last log
      if (node->logInterval + node->lastLogTime <= millis()) {
        //log data
        logline += node->sensor->log();
        if (node->sensor->didLog()) {
          node->lastLogTime = currentTime;
        }
      } else {
        //just print commas
        logline += node->sensor->noLog();
      }
      node = node->next;
    }
    if (globalSettings.logSerial) {
      Serial.println(logline);
    }
    sdLogln(&logline);
  }
}
