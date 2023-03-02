#include <Arduino.h>
#include <Wire.h> //I2C Library
#include <SPI.h> //SPI Library
//#include <SdFat.h> //SD Card Library

#include "SparkFun_u-blox_GNSS_Arduino_Library.h" //http://librarymanager/All#SparkFun_u-blox_GNSS
#include "SparkFun_ISM330DHCX.h" // Click here to get the library: http://librarymanager/All#SparkFun_6DoF_ISM330DHCX
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

class BTSensor {
  private:
    uint8_t address;
    uint8_t dataOutputs = 0;
    bool logStaleData = false;
    String helperText = "";

    bool checkData() {
      //Check if new data is available
      return false;
    }

    void readData() {
      //Read new data
    }

    String logData() {
      //Log data
      return "";
    }

  public:
    BTSensor(uint8_t address) {
      //Sensor constructor
      this->address = address;
    }
    ~BTSensor() {
      //Sensor destructor 
    }

    boolean begin() {
      //Sensor begin
      return false;
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

    void setLogStaleData(boolean logStale) {
      this->logStaleData = logStale;
    }

    String log() {
      static bool dataReady = this->checkData();
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

      return logline;
    }

    String noLog() {
      String logline = "";
      for (int i = 0; i < this->dataOutputs; i++) {
        logline += ",";
      }
      return logline;
    }

    String getHelperText() {
      return this->helperText;
    }

    String getAddressHelperText() {
      String logline = "";
      for (int i = 0; i < this->dataOutputs; i++) {
        logline += "," + String(this->address);
      }
      return logline;
    }

    void setLogStale(boolean logStale) {
      this->logStaleData = logStale;
    }
};

class BT_ISM330DHCX : public BTSensor {
  private:
    SparkFun_ISM330DHCX sensorObj;
    sfe_ism_data_t accelData;
    sfe_ism_data_t gyroData;

    bool checkData() {
      //Check if new data is available
      return (this->sensorObj).checkStatus();
    }

    void readData() {
      //Read new data
      (this->sensorObj).getAccel(&(this->accelData));
      (this->sensorObj).getGyro(&(this->gyroData));
    }

    String logData() {
      String logline = "";
      //Add new data to logline
      logline += "," + String((this->accelData).xData) + ",";
      logline += String((this->accelData).yData) + ",";
      logline += String((this->accelData).zData) + ",";
      logline += String((this->gyroData).xData) + ",";
      logline += String((this->gyroData).yData) + ",";
      logline += String((this->gyroData).zData);
      return logline;
    }

  public:
    BT_ISM330DHCX(uint8_t address) : BTSensor(address) {
      //ISM330DHCX constructor
      this->sensorObj = SparkFun_ISM330DHCX();
      this->setDataOutputs(6);
      this->setHelperText(",AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ");
    }
    ~BT_ISM330DHCX() {
      //ISM330DHCX destructor
    }

    boolean begin() {
      //ISM330DHCX begin
      return (this->sensorObj).begin(this->getAddress());
    }
};

class BT_VL53L1X : public BTSensor {
  private:
    SFEVL53L1X sensorObj;
    uint16_t rangeData = 0;
    bool rangeStatus = false;

    bool checkData() {
      //Check if new data is available
      return (this->sensorObj).checkForDataReady();
    }

    void readData() {
      //Read new data
      this->rangeData = (this->sensorObj).getDistance();
      this->rangeStatus = (this->sensorObj).getRangeStatus();
    }

    String logData() {
      String logline = "";
      //Add new data to logline
      logline += "," + String(this->rangeData);
      if (this->rangeStatus) {
        logline += ",1";
      } else {
        logline += ",0";
      }
      return logline;
    }

  public:
    BT_VL53L1X(uint8_t address) : BTSensor(address) {
      //VL53L1X constructor
      this->sensorObj = SFEVL53L1X();
      this->setLogStaleData(true);
      this->setDataOutputs(2);
      this->setHelperText(",Range,RangeStatus");
    }
    ~BT_VL53L1X() {
      //VL53L1X destructor   
    }

    boolean begin() {
      //VL53L1X begin
      return (this->sensorObj).begin();
    }

};

#define IMU_ISM330DHCX 0x6A
#define IMU_ISM330DHCX_ALT 0x6B
#define LIDAR_VL53L1X 0x29
#define UBX_GNSS 0x42
#define IMU_ICM20948_SPI 0x00

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

  //If logging to serial, begin serial and wait for serial monitor
  if (globalSettings.logSerial) {
    Serial.begin(globalSettings.serialBaudRate);
    while (!Serial) {
      delay(10); // wait for serial port to connect. Needed for native USB port only
    }
  }

  //Turn on I2C
  Wire.begin();

  //Turn on SPI
  SPI.begin();


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
    getInterval(75.0), //75 Hz
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
      }
    }
    node = node->next;
  }

  //Print header:
  node = sensorsHead;
  String header = "\n\n\r";
  //First line: sensor addresses
  while(node != NULL) {
    header += node->sensor->getAddressHelperText();
    node = node->next;
  }
  //Second line: data names
  node = sensorsHead;
  header += "\n\r";
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
    while(node != NULL) {

      //check if enough time has elapsed since last log
      if (node->logInterval + node->lastLogTime <= millis()) {
        //log data
        logline += node->sensor->log();
        node->lastLogTime = currentTime;
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