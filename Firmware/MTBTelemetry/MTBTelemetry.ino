//Define the pin functions -- stolen from OLA code
//Depends on hardware version. This can be found as a marking on the PCB.
//x04 was the SparkX 'black' version.
//v10 was the first red version.
#define HARDWARE_VERSION_MAJOR 1
#define HARDWARE_VERSION_MINOR 0

#define SD_STRESS_TEST 0 //Turn this on to conduct an SD stress test

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
#include <SdFat.h> //SD Card Library

File logFile;
#define NEW_LINE_STR F("\n\r")


#define SD_FAT_TYPE 3 // SD_FAT_TYPE = 0 for SdFat/File, 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_CONFIG SdSpiConfig(PIN_MICROSD_CHIP_SELECT, SHARED_SPI, SD_SCK_MHZ(24)) // 24MHz

#if SD_FAT_TYPE == 1
SdFat32 sd;
File32 sensorDataFile; //File that all sensor data is written to
File32 serialDataFile; //File that all incoming serial data is written to
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile sensorDataFile; //File that all sensor data is written to
ExFile serialDataFile; //File that all incoming serial data is written to
#elif SD_FAT_TYPE == 3
SdFs sd;
FsFile sensorDataFile; //File that all sensor data is written to
FsFile serialDataFile; //File that all incoming serial data is written to
#else // SD_FAT_TYPE == 0
SdFat sd;
File sensorDataFile; //File that all sensor data is written to
File serialDataFile; //File that all incoming serial data is written to
#endif  // SD_FAT_TYPE


#include "SparkFun_u-blox_GNSS_Arduino_Library.h" //http://librarymanager/All#SparkFun_u-blox_GNSS
#include "SparkFun_ISM330DHCX.h" // Click here to get the library: http://librarymanager/All#SparkFun_6DoF_ISM330DHCX
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X
#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "support.h"


//include Apollo RTC
#include "RTC.h"
Apollo3RTC myRTC;

//Sensor Addresses
#define IMU_ISM330DHCX 0x6B
#define IMU_ISM330DHCX_ALT 0x6A
#define LIDAR_VL53L1X 0x29
#define GNSS_UBX 0x42
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
      Serial.print(F("Sensor object created. Address: 0x"));
      Serial.println(address, HEX);
    }
    ~BTSensor() {
      //Sensor destructor 
    }

    virtual bool begin() {
      //Sensor begin
      Serial.println(F("Attempting to begin generic sensor object. Nothing to do."));
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

class BT_ICM20948_SPI : public BTSensor {
  private:
    ICM_20948_SPI *sensorPtr = NULL;
    
    bool checkData() override {
      if (this->sensorPtr->dataReady()) {
        return true;
      } else {
        return false;
      }
    }
    void readData() override {
      (this->sensorPtr->getAGMT());
    }

    String logData() override {
      char buffer1[10];
      char buffer2[10];
      char buffer3[10];
      char buffer4[10];
      char buffer5[10];
      char buffer6[10];
      char buffer7[10];
      char logline[100];

      //Store logged sensor data in buffers
      olaftoa(this->sensorPtr->accX(), buffer1, 3, sizeof(buffer1) / sizeof(char));
      olaftoa(this->sensorPtr->accY(), buffer2, 3, sizeof(buffer2) / sizeof(char));
      olaftoa(this->sensorPtr->accZ(), buffer3, 3, sizeof(buffer3) / sizeof(char));
      olaftoa(this->sensorPtr->gyrX(), buffer4, 3, sizeof(buffer4) / sizeof(char));
      olaftoa(this->sensorPtr->gyrY(), buffer5, 3, sizeof(buffer5) / sizeof(char));
      olaftoa(this->sensorPtr->gyrZ(), buffer6, 3, sizeof(buffer6) / sizeof(char));
      olaftoa(this->sensorPtr->temp(), buffer7, 3, sizeof(buffer7) / sizeof(char));

      //Convert buffers to CSV string
      sprintf(logline, ",%s,%s,%s,%s,%s,%s,%s", buffer1, buffer2, buffer3, buffer4, buffer5, buffer6, buffer7);
      return String(logline);
    }

    void imuPowerOn() {
      pinMode(PIN_IMU_POWER, OUTPUT);
      pin_config(PinName(PIN_IMU_POWER), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
      digitalWrite(PIN_IMU_POWER, HIGH);
    }

    void imuPowerOff() {
      pinMode(PIN_IMU_POWER, OUTPUT);
      pin_config(PinName(PIN_IMU_POWER), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
      digitalWrite(PIN_IMU_POWER, LOW);
    }

    void imuReset(uint8_t offTime, uint8_t breatherTime) {
      //Reset ICM by power cycling it
      this->imuPowerOff();
      delay(offTime);
      this->imuPowerOn();
      delay(breatherTime);
    }

  public:
    BT_ICM20948_SPI(uint8_t address) : BTSensor(address) {
      //Sensor constructor
      this->setDataOutputs(7);
      this->setHelperText(F(",AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Temp"));
      Serial.println(F("ICM20948 (SPI) sensor constructed."));
    }
    ~BT_ICM20948_SPI() {
      //Sensor destructor 
      delete this->sensorPtr;
    }

    bool begin() override {
      //Adapted from Sparkfun Openlog Artemis firmware

      ICM_20948_SPI myICM;

      pinMode(PIN_IMU_POWER, OUTPUT);
      pin_config(PinName(PIN_IMU_POWER), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
      pinMode(PIN_IMU_CHIP_SELECT, OUTPUT);
      pin_config(PinName(PIN_IMU_CHIP_SELECT), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
      digitalWrite(PIN_IMU_CHIP_SELECT, HIGH); //Be sure IMU is deselected

      this->imuReset(10, 25);

      myICM.begin(PIN_IMU_CHIP_SELECT, SPI, 4000000); //Set IMU SPI rate to 4MHz
      if (myICM.status != ICM_20948_Stat_Ok)
      {
        Serial.println("beginIMU: first attempt at myICM.begin failed. myICM.status = " + (String)myICM.status + "\r\n");
        //Try one more time with longer wait
        this->imuReset(40, 200);

        myICM.begin(PIN_IMU_CHIP_SELECT, SPI, 4000000); //Set IMU SPI rate to 4MHz
        if (myICM.status != ICM_20948_Stat_Ok)
        {
          Serial.println("beginIMU: second attempt at myICM.begin failed. myICM.status = " + (String)myICM.status + "\r\n");
          digitalWrite(PIN_IMU_CHIP_SELECT, HIGH); //Be sure IMU is deselected
          this->imuPowerOff();

          //We have failed... for now!
          this->set_running(false);
          return false;
        }
      }
      
      bool success = true;

      //Perform a full startup (not minimal) for non-DMP mode
      ICM_20948_Status_e retval = myICM.startupDefault(false);
      if (retval != ICM_20948_Stat_Ok)
      {
        Serial.println(F("Error: Could not startup the IMU in non-DMP mode!"));
        success = false;
      }
      //Update the full scale and DLPF settings
      retval = myICM.enableDLPF(ICM_20948_Internal_Acc, false);
      if (retval != ICM_20948_Stat_Ok)
      {
        Serial.println(F("Error: Could not configure the IMU Accelerometer DLPF!"));
        success = false;
      }
      retval = myICM.enableDLPF(ICM_20948_Internal_Gyr, false);
      if (retval != ICM_20948_Stat_Ok)
      {
        Serial.println(F("Error: Could not configure the IMU Gyro DLPF!"));
        success = false;
      }
      ICM_20948_dlpcfg_t dlpcfg;
      dlpcfg.a = 7;
      dlpcfg.g = 7;
      retval = myICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlpcfg);
      if (retval != ICM_20948_Stat_Ok)
      {
        Serial.println(F("Error: Could not configure the IMU DLPF BW!"));
        success = false;
      }
      ICM_20948_fss_t FSS;
      FSS.a = 0;
      FSS.g = 0;
      retval = myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), FSS);
      if (retval != ICM_20948_Stat_Ok)
      {
        Serial.println(F("Error: Could not configure the IMU Full Scale!"));
        success = false;
      }

      if(success) {
        this->sensorPtr = &myICM;
        this->set_running(true);
        delay(50);
        return true;
      } else {
        this->imuPowerOff();
        this->set_running(false);
        return false;
      }
    }
};

class BT_UBLOX : public BTSensor {
  private:
    SFE_UBLOX_GNSS *sensorPtr = NULL;
    long lattiude = 0;
    long longitude = 0;
    long altitudeMSL = 0;
    long speed = 0;
    long heading = 0;
    long pDOP = 0;
    char datetime[32];


    bool checkData() override {
      //Check if new data is available
      return true; //you can always talk to the GPS
    }

    void readData() override {
      //Read new data
      this->lattiude = (this->sensorPtr)->getLatitude();
      this->longitude = (this->sensorPtr)->getLatitude();
      this->altitudeMSL = (this->sensorPtr)->getAltitudeMSL();
      this->speed = (this->sensorPtr)->getGroundSpeed();
      this->heading = (this->sensorPtr)->getHeading();
      this->pDOP = (this->sensorPtr)->getPDOP();


      
      //Get Datetime
      char buffer[100];
      sprintf(this->datetime, "%04d-%02d-%02d %02d:%02d:%02d", 
        (this->sensorPtr)->getYear(), 
        (this->sensorPtr)->getMonth(), 
        (this->sensorPtr)->getDay(), 
        (this->sensorPtr)->getHour(), 
        (this->sensorPtr)->getMinute(), 
        (this->sensorPtr)->getSecond());
    }

    String logData() override {
      //Log data
      char logline[100];
      sprintf(logline, ",%s, %ld, %ld, %ld, %ld, %ld, %ld", 
        this->datetime, 
        this->lattiude, 
        this->longitude, 
        this->altitudeMSL, 
        this->speed, 
        this->heading, 
        this->pDOP);
      return String(logline);
    }

  public:
    BT_UBLOX(uint8_t address) : BTSensor(address) {
      //Sensor constructor
      this->setAddress(address);
      this->setHelperText(F(",datetime,lattitude,longitude,altitude,speed,heading,pDOP"));
      this->setDataOutputs(7);
      Serial.println(F("U-blox GNSS sensor constructed."));
    }

    ~BT_UBLOX() {
      //Sensor destructor 
      delete this->sensorPtr;
    }

    bool begin() override {
      //Sensor begin
      Serial.print(F("U-blox begin: 0x"));
      Serial.println(this->getAddress(), HEX);
      
      //create a new instance of the GPS class
      SFE_UBLOX_GNSS *myGPS = new SFE_UBLOX_GNSS;
      
      //attempt to begin communication with the module
      setQwiicPullups(0); //turn off Qwiic Pullups for U-Blox
      bool began = myGPS->begin(qwiic, this->getAddress());
      
      if(!began) { //Something went wrong
        Serial.println(F("U-blox begin failed."));
      }
      else {
        Serial.println(F("U-blox online!")); 
        this->sensorPtr = myGPS;
      }

      //clean up
      setQwiicPullups(1); //turn Qwiic back on 
      //TODO make the above use a global setting.  
      this->set_running(began);
      return began;

    }
};

class BT_ISM330DHCX : public BTSensor {
  private:
    SparkFun_ISM330DHCX *sensorPtr;
    sfe_ism_data_t *accelData = NULL;
    sfe_ism_data_t *gyroData = NULL;

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

      sfe_ism_data_t* oldAccelData = this->accelData;
      sfe_ism_data_t* oldGyroData = this->gyroData;
      
      this->accelData = accelData;
      this->gyroData = gyroData;

      delete oldAccelData;
      delete oldGyroData;
    }

    String logData() override {
      char buffer1[10];
      char buffer2[10];
      char buffer3[10];
      char buffer4[10];
      char buffer5[10];
      char buffer6[10];
      char logline[100];

      sfe_ism_data_t accelData = *(this->accelData);
      sfe_ism_data_t gyroData = *(this->gyroData);

      //Add new data to logline
      olaftoa(accelData.xData, buffer1, 3, sizeof(buffer1) / sizeof(char));
      olaftoa(accelData.yData, buffer2, 3, sizeof(buffer2) / sizeof(char));
      olaftoa(accelData.zData, buffer3, 3, sizeof(buffer3) / sizeof(char));
      olaftoa(gyroData.xData, buffer4, 3, sizeof(buffer4) / sizeof(char));
      olaftoa(gyroData.yData, buffer5, 3, sizeof(buffer5) / sizeof(char));
      olaftoa(gyroData.zData, buffer6, 3, sizeof(buffer6) / sizeof(char));
      sprintf(logline, ",%s,%s,%s,%s,%s,%s", buffer1, buffer2, buffer3, buffer4, buffer5, buffer6);

      String loglineString = String(logline);
      return loglineString;
    }

  public:
    BT_ISM330DHCX(uint8_t address) : BTSensor(address) {  
      //ISM330DHCX constructor
      this->setDataOutputs(6);
      this->setHelperText(F(",AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ"));
      Serial.println(F("ISM330DHCX sensor constructed."));
    }
    ~BT_ISM330DHCX() {
      //ISM330DHCX destructor
      delete this->sensorPtr;
    }

    bool begin() override {
      //ISM330DHCX begin
      Serial.print(F("ISM330DHCX begin: 0x"));
      Serial.println(this->getAddress(), HEX);
      SparkFun_ISM330DHCX *myISM = new SparkFun_ISM330DHCX;

      int tries = 0;

      while( !(myISM->begin(qwiic, this->getAddress())) ){
        Serial.println(F("Did not begin. Tying Again"));
        delay(1000);
        if (tries > 5) {
          Serial.println(F("Failed to begin ISM330DHCX"));
          return false;
        } else {
          tries++;
        }
      }

      // Reset the device to default settings. This if helpful is you're doing multiple
      // uploads testing different settings. 
      Serial.print(F("Resetting device."));
      myISM->deviceReset();

      // Wait for it to finish reseting
      while( !(myISM->getDeviceReset()) ){ 
        delay(10);
        Serial.print(".");
      } 
      Serial.println(F(" Done."));

      Serial.println(F("Applying settings."));
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
      Serial.print(F("Waiting for data."));
      while(!(myISM->checkStatus())) {
        delay(100);
        Serial.print(".");
      }
      Serial.println(F(" Done!"));

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
      this->setHelperText(F(",Range,RangeStatus"));
      Serial.println(F("VL53L1X sensor constructed."));
    }
    ~BT_VL53L1X() {
      //VL53L1X destructor   
    }

    bool begin() override {
      //VL53L1X begin
      Serial.print(F("VL53L1X begin: 0x"));
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

typedef struct {
  bool sdOnline = false;
} TrackerStatus;

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
TrackerStatus globalStatus;

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

void microSDPowerOn()
{
  pinMode(PIN_MICROSD_POWER, OUTPUT);
  pin_config(PinName(PIN_MICROSD_POWER), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
  digitalWrite(PIN_MICROSD_POWER, LOW);
}

void microSDPowerOff()
{
  pinMode(PIN_MICROSD_POWER, OUTPUT);
  pin_config(PinName(PIN_MICROSD_POWER), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
  digitalWrite(PIN_MICROSD_POWER, HIGH);
}

void beginSD(bool silent)
{
  pinMode(PIN_MICROSD_POWER, OUTPUT);
  pin_config(PinName(PIN_MICROSD_POWER), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
  pinMode(PIN_MICROSD_CHIP_SELECT, OUTPUT);
  pin_config(PinName(PIN_MICROSD_CHIP_SELECT), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
  digitalWrite(PIN_MICROSD_CHIP_SELECT, HIGH); //Be sure SD is deselected

  // If the microSD card is present, it needs to be powered on otherwise the IMU will fail to start
  // (The microSD card will pull the SPI pins low, preventing communication with the IMU)

  // For reasons I don't understand, we seem to have to wait for at least 1ms after SPI.begin before we call microSDPowerOn.
  // If you comment the next line, the Artemis resets at microSDPowerOn when beginSD is called from wakeFromSleep...
  // But only on one of my V10 red boards. The second one I have doesn't seem to need the delay!?
  delay(5);

  microSDPowerOn();

  //Max power up time is 250ms: https://www.kingston.com/datasheets/SDCIT-specsheet-64gb_en.pdf
  //Max current is 200mA average across 1s, peak 300mA
  delay(10);

  if (sd.begin(SD_CONFIG) == false) // Try to begin the SD card using the correct chip select
  {
    Serial.println("SD init failed (first attempt). Trying again...\r\n");
    delay(250);
    if (sd.begin(SD_CONFIG) == false) // Try to begin the SD card using the correct chip select
    {
      if (!silent)
      {
        Serial.println("SD init failed (second attempt). Is card present? Formatted?");
        Serial.println("Please ensure the SD card is formatted correctly using https://www.sdcard.org/downloads/formatter/");
      }
      digitalWrite(PIN_MICROSD_CHIP_SELECT, HIGH); //Be sure SD is deselected
      globalStatus.sdOnline = false;
      return;
    }
  }

  //Change to root directory. All new file creation will be in root.
  if (sd.chdir() == false)
  {
    if (!silent)
    {
      Serial.println("SD change directory failed");
    }
    globalStatus.sdOnline = false;
    return;
  }

  globalStatus.sdOnline = true;
}

void bothLog(String log) {
  if (globalSettings.logSerial) {
    Serial.print(log);
  }
  if (globalSettings.logSD && globalStatus.sdOnline && logFile.isOpen()) {
    logFile.print(log);
  } else {
    if(globalSettings.logSD) {
      Serial.println("Cannot log to SD. SD card not online");
      globalSettings.logSD = false;
    }
  }
}

void bothLogln(String logline) {
  bothLog(logline);
  bothLog("\n\r");
}

#if(SD_STRESS_TEST == 1)
void sdStressTest(int maxChars) {
  srand(time(NULL));
  Serial.println("Begin SD stress test.");
  String logline = "";
  for (int i = 0; i < maxChars; i++) {

    char randomChar;
    int randomNum = rand() % 29;  // generate a random number between 0 and 28
    if (randomNum < 26) {
        // generate a random lowercase or uppercase letter
        randomChar = randomNum < 13 ? 'a' + randomNum : 'A' + randomNum - 13;
    } else {
        Serial.print("Added escape char this line: ");
        // generate a random escape character
        switch (randomNum) {
            case 26:
                randomChar = '\n';
                break;
            case 27:
                randomChar = '\r';
                break;
            case 28:
                randomChar = '\t';
                break;
        }
    }
    logline.concat(randomChar);
    Serial.println(logline);
    sdLogln(logline);
    
  }
}
#endif

/*
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*/

static unsigned long lastLogTime;

char logBuffer[128];

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

  //Begin SD card:
  beginSD(false);

  //create a logfile
  int fileNumber = 0;
  String logfilename;
  do {
    logfilename = "log" + String(fileNumber) + ".csv";
    fileNumber++;
  } while(sd.exists(logfilename));

  logFile = sd.open(logfilename, O_CREAT | O_APPEND | O_WRITE);
  Serial.println("Opened logfile at: " + logfilename);

  #if(SD_STRESS_TEST == 1)
  //Conduct SD Card stress test
  sdStressTest(200);
  #endif

  //Detect and initialize sensors
  //TODO: Autodetect connected sensors
  
  //IMU #1
  addSensor(new SensorNode {
    getInterval(150.0), //150 Hz
    IMU_ISM330DHCX,
    new BT_ISM330DHCX(IMU_ISM330DHCX),
    millis(),
    NULL
  });

  //IMU #2
  addSensor(new SensorNode {
    getInterval(150.0), //150 Hz
    IMU_ISM330DHCX_ALT,
    new BT_ISM330DHCX(IMU_ISM330DHCX_ALT),
    millis(),
    NULL
  });

  //Lidar sensor
  addSensor(new SensorNode {
    getInterval(50.0), //50 Hz
    LIDAR_VL53L1X,
    new BT_VL53L1X(LIDAR_VL53L1X),
    millis(),
    NULL
  });

  //GNSS sensor
  addSensor(new SensorNode {
    getInterval(0.5), //0.5Hz
    GNSS_UBX,
    new BT_UBLOX(GNSS_UBX),
    millis(),
    NULL
  });


  /*
  //Onboard IMU
  addSensor(new SensorNode {
    getInterval(150.0), //150 Hz
    IMU_ICM20948_SPI,
    new BT_ICM20948_SPI(IMU_ICM20948_SPI),
    millis(),
    NULL
  });
  */

  //begin sensors:
  SensorNode *node = sensorsHead;
  while(node != NULL) {
    if (node->sensor->begin()) {
      if (globalSettings.logSerial) {
        Serial.print(F("Sensor at address 0x"));
        Serial.print(node->address, HEX);
        Serial.println(F(" initialized."));
      }
    } else {
      if (globalSettings.logSerial) {
        Serial.print(F("Sensor at address 0x"));
        Serial.print(node->address, HEX);
        Serial.println(F(" failed to initialize."));

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

  Serial.println(F("Done initializing sensors."));

  //Print header:
  node = sensorsHead;
  bothLog(F("Biketracker"));
  //First line: sensor addresses
  while(node != NULL) {
    bothLog(node->sensor->getAddressHelperText());
    node = node->next;
  }

  bothLog(NEW_LINE_STR);

  //Second line: data names
  node = sensorsHead;
  bothLog(F("Time_Millis"));
  while(node != NULL) {
    bothLog(node->sensor->getHelperText());
    node = node->next;
  }
  bothLog(NEW_LINE_STR);

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
    sprintf(logBuffer, "%lu", currentTime);
    bothLog(logBuffer);
    while(node != NULL) {

      //check if enough time has elapsed since last log
      if (node->logInterval + node->lastLogTime <= millis()) {
        //request data logging (the sensor might refuse if the data is not ready)
        bothLog(node->sensor->log());
        if (node->sensor->didLog()) {
          if(millis() - currentTime > node->logInterval) {
            //count from the end of logging to avoid bogging
            node->lastLogTime = millis();
          } else {
            //default to counting from the beginning of logging
            node->lastLogTime = currentTime;
          }
        }
      } else {
        //just print commas
        sprintf(logBuffer, "%s", node->sensor->noLog());
        bothLog(logBuffer);
      }
      node = node->next;
    }
    bothLog(NEW_LINE_STR);
  }
}
