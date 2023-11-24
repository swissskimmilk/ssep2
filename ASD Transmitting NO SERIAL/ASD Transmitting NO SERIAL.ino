// Basic demo for Computerized Utility Module
// A few red text while uploading is normal, it should say in white text at end completed
// If there is an error that says wrong boot mode, unplug and plug in microcontroller
// 
/*
Compile options:
ESP32S3 Dev Module
USB CDC on
240Mhz Wifi
Debug none
USB DFU disabled
Erase flash disabled
Events + Arduino on Core 0
Flash QIO 80mhz
Flash size 8mb
JTAG disabled
USB MSC disabled
Partition 8mb
PSRAM disabled
upload mode uart0 / cdc
upload speed 921600
usb mode hardware cdc



To Do:
SD Card Read/Write + New txt each launch
Optimize SD Card print lines

LoRa transmit important data

Arming web page


Startup Calibration:
ENSURE USB AND LIPO ARE NOT BOTH CONNECTED
When device is powered on, a red light near the esp32 will turn on.
Immediately place device at angle it will be mounted in rocket assuming vertical flight.
After 10 seconds, the device will calibrate. After a few more seconds the light will turn off.
When calibration is done. The light will blink then turn off.
IF THE LED DOES NOT GO OFF AFTER 20 SECONDS, THE SD CARD IS NOT PROPERLY SEATED.
YOU MUST RESEAT THE SD CARD AND UNPLUG AND REPLUG THE DEVICE

Sensor Calibration:
Remove device from rocket, move in 4-6 unique orientations and hold in that orientation for 2 seconds
Rotate around Z axis 360 degrees
Rotate 180 degrees and back to beginning position, rotate for 2 seconds for all 3 axes
Wait until the blinking light blinks at 4 times per second (really fast vs 1 times per second standard)
This may take over 3 minutes to be stable blinking
Install device into rocket. Calibration is done
*/

#include <String.h>
#include <Adafruit_BNO08x.h>
#include <Ra01S.h>
#include <EEPROM.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>
#include <MS5803_01.h>
#include <SparkFun_u-blox_GNSS_v3.h> 

#define BNO08X_CS 14
#define BNO08X_INT 42
#define BNO08X_RESET 41
#define SDA 3
#define SCL 4
#define MOSI 11
#define CLK 12
#define MISO 13
#define SDCS 10
#define LED 6
#define eeAddress 0

#define RF_FREQUENCY                                915000000 // Hz  center frequency
#define TX_OUTPUT_POWER                             22        // dBm tx output power
#define LORA_SPREADING_FACTOR                       6         // spreading factor [SF5..SF12]
#define LORA_BANDWIDTH                              5         // bandwidth
                                                              // 2: 31.25Khz
                                                              // 3: 62.5Khz
                                                              // 4: 125Khz
                                                              // 5: 250KHZ
                                                              // 6: 500Khz 
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]

#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_PAYLOAD_LENGTH                          0         // 0: Variable length packet (explicit header)
                                                              // 1..255  Fixed length packet (implicit header)

SX126x  lora(15,               //Port-Pin Output: SPI select
             21,               //Port-Pin Output: Reset 
             39               //Port-Pin Input:  Busy
             );

#define INFO_STRING_ID 0x01
#define GNSS_ID 2
#define BARO_ID 3
#define SH2_ROT_ID 4
#define SH2_ACCEL_ID 5
#define UBX_ID 6
#define SATS_ID 7
#define TIME_ID 8
#define SH2_GYRO_ID 9
#define RESET_ID 12
#define ACCEL_ID 11

uint8_t currOutput[255];
int currIndex = 0;

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;
unsigned long previousMicros = 0;
bool collectData = false;
double pressure_baseline;
unsigned long lastTime = 0;

File myFile;


MS_5803 baro1 = MS_5803(4096, 0x77);

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

//TwoWire myWireBus(1);
/* Assign a unique ID to this sensor at the same time */
/* Uncomment following line for default Wire bus      */
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);

SFE_UBLOX_GNSS myGNSS;

 

void setup(void) {

  // Serial.begin(115200);

  int16_t ret = lora.begin(RF_FREQUENCY,              //frequency in Hz
                           TX_OUTPUT_POWER);          //tx power in dBm
  if (ret != ERR_NONE) while(1) {delay(1);}

  lora.LoRaConfig(LORA_SPREADING_FACTOR, 
                  LORA_BANDWIDTH, 
                  LORA_CODINGRATE, 
                  LORA_PREAMBLE_LENGTH, 
                  LORA_PAYLOAD_LENGTH, 
                  true,               //crcOn  
                  false);             //invertIrq

  // GOTTA MESS WITH THE PINS PROB 
  if (!SD.begin(SDCS)) {
    delay(100);
    while (1) {
      delay(100);
    }
  }
  myFile = SD.open("/star.txt", FILE_APPEND);
  if (SD.exists("/star.txt")) {
    //Serial.println("File created successfully!");
  } 
  else {
    while(1) {
      delay(100);
    }
  }
  if (myFile) {
    myFile.print("Writing to star.txt...");    
    myFile.println("Space Technologies and Rocketry");
    myFile.println("Computerized Utility Module");
    myFile.println("Version 47 Prototype 2 Code");
    myFile.println("Designed, Assembled, and Coded by Conor Van Bibber with help from Aidan and Rohith");
    myFile.println("Launch Number: 01");
    myFile.close();
    
  } else {
    // if the file didn't open, print an error:
    while(1) {
      delay(100);
    }
  }  
  digitalWrite(SDCS, HIGH);
                  
  pinMode(BNO08X_INT, INPUT);
  pinMode(LED,OUTPUT);
  digitalWrite(LED,HIGH);
  SPI.begin(CLK, MISO, MOSI);
  
  setID(INFO_STRING_ID);
  addToOutput("STAR Flight Computer Test");
  outputData();
  setID(INFO_STRING_ID);
  addToOutput("Designed by Conor Van Bibber");
  outputData();

  Wire.begin(3, 4);
  
  Wire.setClock(400000);
  delay(2000);
  baro1.initializeMS_5803(false); //this needs to be first to give calibration time

  // Try to initialize!
  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    setID(INFO_STRING_ID);
    addToOutput("Failed to find BNO08x chip");
    outputData();
    while (1) {
      delay(10);
    }
  }

  setID(INFO_STRING_ID);
  addToOutput("BNO08x Found!");
  outputData();
  delay(1000);

  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL375 ... check your connections */
    setID(INFO_STRING_ID);
    addToOutput("Ooops, no ADXL375 detected ... Check your wiring!");
    outputData();
    while(1) {
      delay(10);
    }
  }

  if (myGNSS.begin() == false) {
    setID(INFO_STRING_ID);
    addToOutput("Failed to Start GNSS Module");
    outputData();
    while (1) {
      delay(10);
    }
  }
  myGNSS.setI2COutput(COM_TYPE_UBX);
  
  myGNSS.newCfgValset(VAL_LAYER_RAM); // Create a new Configuration Interface VALSET message. Apply the changes in RAM only (not BBR).

  // While the module is _locking_ to GNSS time, make it generate 2kHz
  myGNSS.addCfgValset(UBLOX_CFG_TP_FREQ_TP1, 1); // Set the frequency to 2000Hz
  myGNSS.addCfgValset(UBLOX_CFG_TP_DUTY_TP1, 50.0); // Set the pulse ratio / duty to 33.333% to produce 33.333:66.666 mark:space

  // When the module is _locked_ to GNSS time, make it generate 1kHz
  myGNSS.addCfgValset(UBLOX_CFG_TP_FREQ_LOCK_TP1, 4); // Set the frequency to 1000Hz
  myGNSS.addCfgValset(UBLOX_CFG_TP_DUTY_LOCK_TP1, 50.0); // Set the pulse ratio / duty to 50% to produce 50:50 mark:space

  myGNSS.addCfgValset(UBLOX_CFG_TP_TP1_ENA, 1); // Make sure the enable flag is set to enable the time pulse. (Set to 0 to disable.)
  myGNSS.addCfgValset(UBLOX_CFG_TP_USE_LOCKED_TP1, 1); // Tell the module to use FREQ while locking and FREQ_LOCK when locked to GNSS time
  myGNSS.addCfgValset(UBLOX_CFG_TP_PULSE_DEF, 1); // Tell the module that we want to set the frequency (not the period). PERIOD = 0. FREQ = 1.
  myGNSS.addCfgValset(UBLOX_CFG_TP_PULSE_LENGTH_DEF, 0); // Tell the module to set the pulse ratio / duty (not the pulse length). RATIO = 0. LENGTH = 1.
  myGNSS.addCfgValset(UBLOX_CFG_TP_POL_TP1, 1); // Tell the module that we want the rising edge at the top of second. Falling Edge = 0. Rising Edge = 1.
  if (myGNSS.sendCfgValset() == false)
  {
    setID(INFO_STRING_ID);
    addToOutput("VALSET failed!");
    outputData();
  }
  else
  {
    setID(INFO_STRING_ID);
    addToOutput("Success!");
    outputData();
  }
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
  myGNSS.setNavigationFrequency(10);
  if (myGNSS.setDynamicModel(DYN_MODEL_AIRBORNE4g, VAL_LAYER_RAM_BBR) == false) // Set the dynamic model to PORTABLE in both RAM and BBR
  {
    setID(INFO_STRING_ID);
    addToOutput("*** Warning: setDynamicModel failed ***");
    outputData();
  }
  else
  {
    setID(INFO_STRING_ID);
    addToOutput("Dynamic platform model changed successfully!");
    outputData();
  }
  uint8_t newDynamicModel = myGNSS.getDynamicModel(VAL_LAYER_RAM); // Read the value from the RAM layer. (We could choose DEFAULT instead.)
  if (newDynamicModel == DYN_MODEL_UNKNOWN)
  {
    setID(INFO_STRING_ID);
    addToOutput("*** Warning: getDynamicModel failed ***");
    outputData();
  }
  else
  {
    setID(INFO_STRING_ID);
    addToOutput(String("The new dynamic model is: ") + String(newDynamicModel)); 
    outputData();
  }
  uint16_t measRate = myGNSS.getMeasurementRate(); //Get the measurement rate of this module
  setID(INFO_STRING_ID);
  addToOutput(String("Current measurement interval (ms): ") + String(measRate));
  outputData();
  uint16_t navRate = myGNSS.getNavigationRate(); //Get the navigation rate of this module
  setID(INFO_STRING_ID);
  addToOutput(String("Current navigation ratio (cycles): ") + String(navRate));
  outputData();
  if (myGNSS.setMeasurementRate(100, VAL_LAYER_RAM) == false) // Change the rate in RAM only - don't save to BBR
  {
    setID(INFO_STRING_ID);
    addToOutput("Could not set the measurement rate. Freezing.");
    outputData();
    while (1){
      delay(10);
    }
  }
  myGNSS.setI2CpollingWait(25);
   if (myGNSS.setNavigationRate(3, VAL_LAYER_RAM) == false) // Change the rate in RAM only - don't save to BBR
  {
    setID(INFO_STRING_ID);
    addToOutput("Could not set the navigation rate. Freezing.");
    outputData();
    while (1) {
      delay(10);
      }
  }

  
  setID(INFO_STRING_ID);
  measRate = myGNSS.getMeasurementRate(); //Get the measurement rate of this module
  addToOutput(String("New measurement interval (ms): ") + String(measRate));

  navRate = myGNSS.getNavigationRate(); //Get the navigation rate of this module
  addToOutput(" | New navigation ratio (cycles): " + String(navRate));

  addToOutput(" | PVT data will be sent every " + String(measRate * navRate / 1000) + " seconds");
  outputData();
  
  lastTime = millis();
  myGNSS.setAutoPVTcallbackPtr(&printPVTdata);
  myGNSS.setAutoNAVSATcallbackPtr(&newNAVSAT); 

  

  for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
    setID(INFO_STRING_ID);
    addToOutput("Part " + String(bno08x.prodIds.entry[n].swPartNumber) + ": Version: " + String(bno08x.prodIds.entry[n].swVersionMajor) + "." +
                String(bno08x.prodIds.entry[n].swVersionMinor) + "." + String(bno08x.prodIds.entry[n].swVersionPatch) + " Build " + String(bno08x.prodIds.entry[n].swBuildNumber));
    outputData();
  }

  setReports();

  accel.setDataRate(ADXL343_DATARATE_800_HZ);
  accel.printSensorDetails();
  int8_t x_offset, y_offset, z_offset;
  accel.getTrimOffsets(&x_offset, &y_offset, &z_offset);
  setID(INFO_STRING_ID);
  addToOutput("Current trim offsets: " + String(x_offset) + ", " + String(y_offset) + ", " + String(z_offset));
  outputData();
  accel.setTrimOffsets(0, 0, 0);
  
  setID(INFO_STRING_ID);
  addToOutput("Hold accelerometer flat to set offsets to 0, 0, and -1g...");
  outputData();
  delay(10000);
  int16_t x, y, z;
  x = accel.getX();
  y = accel.getY();
  z = accel.getZ();
  setID(INFO_STRING_ID);
  addToOutput("Raw X: " + String(x) + " Y: " + String(y) + " Z: " + String(z) + " counts"); 
  outputData();
   // the trim offsets are in 'multiples' of 4, we want to round, so we add 2
  accel.setTrimOffsets(-(x+2)/4, 
                       -(y+2)/4, 
                       -(z-20+2)/4);  // Z should be '20' at 1g (49mg per bit)

  digitalWrite(LED,LOW);
  accel.getTrimOffsets(&x_offset, &y_offset, &z_offset);
  setID(INFO_STRING_ID);
  addToOutput("Current trim offsets: " + String(x_offset) + ", " + String(y_offset) + ", " + String(z_offset));
  outputData();
  delay(1000); 
  digitalWrite(LED,HIGH); 
  delay(5000);
  baro1.readSensor();
  pressure_baseline = baro1.pressure();
  setID(INFO_STRING_ID);
  addToOutput(String("Ground Pressure (mbar):  ") + String(pressure_baseline));
  addToOutput(" Calibration setup done, reading sensors.");
  outputData();
  delay(100);

  digitalWrite(LED,LOW);

  //myFile.close();
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  setID(INFO_STRING_ID);
  addToOutput("Setting desired reports");
  outputData();
  // if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
  //   addThenOutput("Could not enable accelerometer");
  // } 
  /*
  if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 2500)) { //max 400hz
    setID(INFO_STRING_ID);
    addToOutput("Could not enable gyroscope");
    outputData();
  }
  */
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED,2500)) { //max 400hz
    setID(INFO_STRING_ID);
    addToOutput("Could not enable gyroscope");
    outputData();
  }
 /* if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
    addThenOutput("Could not enable magnetic field calibrated");
  } */
  
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION, 2500)) { //max 400hz
    setID(INFO_STRING_ID);
    addToOutput("Could not enable linear acceleration");
    outputData();
  }
  
 /* if (!bno08x.enableReport(SH2_GRAVITY)) {
    addThenOutput("Could not enable gravity vector");
  }
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
    addThenOutput("Could not enable rotation vector");
  }
  
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    addThenOutput("Could not enable game rotation vector");
  }

  if (!bno08x.enableReport(SH2_STABILITY_CLASSIFIER)) {
    addThenOutput("Could not enable stability classifier");
  }
  
  if (!bno08x.enableReport(SH2_SHAKE_DETECTOR)) {
    addThenOutput("Could not enable shake detector");
  } */

}
/*
Quaternion to Euler is a very slow calculation so we should do this after uploading the data.
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}
void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}
*/
// void printVariableNoUnits(char* name, long variable) {
//   Serial.print(" "); Serial.print(F(name)); Serial.print(": ");
//   Serial.print(variable);
// }

// void printVariableAndUnits(char* name, long variable, char* units) {
//   Serial.print(" "); Serial.print(F(name)); Serial.print(": ");
//   Serial.print(variable);
//   Serial.print(" ("); Serial.print(F(units)); Serial.print(") ");
// }

void loop() {
  long currentMicros = micros();
  myGNSS.checkUblox(); // Check for the arrival of new data and process it.
  myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.
  if (myGNSS.getPVT()) //Check for new Position, Velocity, Time data. getPVT returns true if new data is available.
  {    
    long time = myGNSS.getNAVTIMEUTC();    
    setID(GNSS_ID);
    addToOutput(time); 
    long latitude = myGNSS.getLatitude();
    addToOutput(latitude);
      
    long longitude = myGNSS.getLongitude();
    addToOutput(longitude);

    //Calculate the interval since the last message
    addToOutput(((float)(millis() - lastTime)) / (float)1000.0);

    long altitudeMSL = myGNSS.getAltitudeMSL();
    addToOutput(altitudeMSL);

    uint8_t fixType = myGNSS.getFixType();
    //addToOutput(F(" Fix: "));
    // if(fixType == 0) addToOutput(F("No fix"));
    // else if(fixType == 1) addToOutput(F("Dead reckoning"));
    // else if(fixType == 2) addToOutput(F("2D"));
    // else if(fixType == 3) addToOutput(F("3D"));
    // else if(fixType == 4) addToOutput(F("GNSS + Dead reckoning"));
    // else if(fixType == 5) addToOutput(F("Time only"));
    addToOutput(fixType); // int8 

    lastTime = millis(); //Update lastTime
  } 
  //Read all i2c sensors before spi stuff
  baro1.readSensor();
  setID(BARO_ID);
  //addToOutput("Pressure = ");
  addToOutput(baro1.pressure()); // float
  //addToOutput("Temperature = ");
  addToOutput(baro1.temperature()); //float

  sensors_event_t event;
  accel.getEvent(&event);
  
  setID(ACCEL_ID);
  addToOutput(event.acceleration.x); 
  addToOutput(event.acceleration.y); 
  addToOutput(event.acceleration.z);
 
  // Check the state of the interrupt pin
  if (digitalRead(BNO08X_INT) == LOW) {
    // If the interrupt pin is LOW, set the flag to collect data
    collectData = true;
    //begin collecting BNO data
    // if (!bno08x.getSensorEvent(&sensorValue)) {
    //   Serial.println("The thing that never happens happened");
    //   return; // NEVER HAPPENS
    // }  
    bno08x.getSensorEvent(&sensorValue);
    // Serial.println(sensorValue.sensorId);
    switch (sensorValue.sensorId) {
      // case SH2_ACCELEROMETER: //accelerometer including gravity, 3 axis defined by sensor orientation, m/s^2
      //   setID(SH2_ACCEL_ID);
      //   addToOutput("Accelerometer - x: ");
      //   addToOutput(sensorValue.un.accelerometer.x);
      //   addToOutput(sensorValue.un.accelerometer.y);
      //   addToOutput(sensorValue.un.accelerometer.z);
      //   break; 
      // case SH2_GYROSCOPE_CALIBRATED: //angular velocity 3 axis defined by sensor orientation, rad/s
      //   setID(SH2_GYRO_ID);
      //   addToOutput(sensorValue.un.gyroscope.x);
      //   addToOutput(sensorValue.un.gyroscope.y);
      //   addToOutput(sensorValue.un.gyroscope.z);
      //   break;
    /* case SH2_MAGNETIC_FIELD_CALIBRATED: // magnetic field 3 axis def by sensor orientation, uT
        addToOutput("Magnetic Field - x: ");
        addToOutput(sensorValue.un.magneticField.x);
        addToOutput(" y: ");
        addToOutput(sensorValue.un.magneticField.y);
        addToOutput(" z: ");
        addThenOutput(sensorValue.un.magneticField.z);
        break; */
      // case SH2_ARVR_STABILIZED_RV: //VR hyperstabilized rotation vector, slow, kalman filtered
      // {
      //   float qr = sensorValue.un.arvrStabilizedRV.real;
      //   float qi = sensorValue.un.arvrStabilizedRV.i;
      //   float qj = sensorValue.un.arvrStabilizedRV.j;
      //   float qk = sensorValue.un.arvrStabilizedRV.k;
      //   break; 
      // }
      
      	// quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        // setID(SH2_ROT_ID);
        // addToOutput(ypr.yaw);
        // addToOutput(ypr.pitch);
        // addToOutput(ypr.roll);
        // break;  
      case SH2_LINEAR_ACCELERATION: // same as accelerometer but without gravity (0,0,0 at rest)
        setID(SH2_ACCEL_ID);  
        addToOutput(sensorValue.un.linearAcceleration.x);
        addToOutput(sensorValue.un.linearAcceleration.y);
        addToOutput(sensorValue.un.linearAcceleration.z);
        break;
      /* case SH2_GRAVITY: //gravity separated value, can be used to determine rocket orientation, m/s^2
        addToOutput("Gravity - x: ");
        addToOutput(sensorValue.un.gravity.x);
        addToOutput(" y: ");
        addToOutput(sensorValue.un.gravity.y);
        addToOutput(" z: ");
        addThenOutput(sensorValue.un.gravity.z);
        break; */
      /* case SH2_ROTATION_VECTOR: //quaternion using all sensors and magnetic north and gravity for kalman filtered orientation, 
        addToOutput("Rotation Vector - r: ");
        addToOutput(sensorValue.un.rotationVector.real);
        addToOutput(" i: ");
        addToOutput(sensorValue.un.rotationVector.i);
        addToOutput(" j: ");
        addToOutput(sensorValue.un.rotationVector.j);
        addToOutput(" k: ");
        addThenOutput(sensorValue.un.rotationVector.k);
        break;*/
    }
  } 
  else {
    // If the interrupt pin is not LOW, set the flag to stop collecting data
    // Serial.println("outputting");
    // collectData = false;
    // outputData();
    
   //begin writing to SD Card


  }
  // Serial.println("outputting");
  collectData = false;
  outputData();
  if (bno08x.wasReset()) {
    setID(RESET_ID);
    addToOutput((uint8_t)1);
    outputData();
    setReports();
  }


  // Prints acceleration very inaccurate in m/s^2 with gravity included +-200g
  previousMicros = currentMicros;
  // Print the time indicator
  setID(TIME_ID);
  addToOutput(currentMicros);
}

void newNAVSAT(UBX_NAV_SAT_data_t *ubxDataStruct)
{
  setID(SATS_ID);
  addToOutput(ubxDataStruct->header.numSvs);
  outputData();

  // Just for giggles, print the signal strength for each SV as a barchart
  // for (uint16_t block = 0; block < ubxDataStruct->header.numSvs; block++) // For each SV
  // {
  //   switch (ubxDataStruct->blocks[block].gnssId) // Print the GNSS ID
  //   {
  //     case 0:
  //       addToOutput(F("GPS"));
  //     break;
  //     case 1:
  //       addToOutput(F("SBAS"));
  //     break;
  //     case 2:
  //       addToOutput(F("Galileo"));
  //     break;
  //     case 3:
  //       addToOutput(F("BeiDou"));
  //     break;
  //     case 4:
  //       addToOutput(F("IMES"));
  //     break;
  //     case 5:
  //       addToOutput(F("QZSS"));
  //     break;
  //     case 6:
  //       addToOutput(F("GLONASS"));
  //     break;
  //     default:
  //       addToOutput(F("UNKNOWN "));
  //     break;      
  //   }
    
  //   addToOutput(ubxDataStruct->blocks[block].svId); // Print the SV ID
  // }
  
}
void printPVTdata(UBX_NAV_PVT_data_t *ubxDataStruct)
{
    setID(UBX_ID);
    uint8_t hms = ubxDataStruct->hour; // Print the hours
    //if (hms < 10) addToOutput("0"); // Print a leading zero if required
    addToOutput(hms); // hours
    hms = ubxDataStruct->min; // Print the minutes
    //if (hms < 10) addToOutput(F("0")); // Print a leading zero if required
    addToOutput(hms); // minutes
    hms = ubxDataStruct->sec; // Print the seconds
    //if (hms < 10) addToOutput(F("0")); // Print a leading zero if required
    addToOutput(hms); // seconds 
    long millisecs = ubxDataStruct->iTOW % 1000; // Print the milliseconds
    //if (millisecs < 100) addToOutput(F("0")); // Print the trailing zeros correctly
    //if (millisecs < 10) addToOutput(F("0"));
    addToOutput(millisecs);

    long latitude = ubxDataStruct->lat; // Print the latitude
    addToOutput(latitude);

    long longitude = ubxDataStruct->lon; // Print the longitude
    addToOutput(longitude);

    long altitude = ubxDataStruct->hMSL; // Print the height above mean sea level
    addToOutput(altitude);

    long NorthVel = ubxDataStruct->velN;
    addToOutput(NorthVel);
    long EastVel = ubxDataStruct->velE;
    addToOutput(EastVel);
    long DownVel = ubxDataStruct->velD;
    addToOutput(DownVel);
    long Groundspeed = ubxDataStruct->gSpeed;
    addToOutput(Groundspeed);
    long VerticalAcc = ubxDataStruct->vAcc;
    addToOutput(VerticalAcc);
    long HorizontalAcc = ubxDataStruct->hAcc;
    addToOutput(HorizontalAcc);
    long SpeedAcc = ubxDataStruct->sAcc;
    addToOutput(SpeedAcc);
    outputData();

}

void setID(uint8_t identifier){
  currOutput[currIndex] = identifier;
  currIndex++;
}

void addToOutput(uint8_t output) {
  currOutput[currIndex] = output;
  currIndex++;
}

void addToOutput(float output) {
  memcpy(currOutput + currIndex, &output, 4);
  currIndex += 4;
}

void addToOutput(long output) {
  memcpy(currOutput + currIndex, &output, 4);
  currIndex += 4;
}

void addToOutput(String output) {
  strcat((char *)(currOutput + currIndex), output.c_str());
  currIndex += output.length();
}

// // this is buggy as hell cuz null
// void addToOutput(char * output) {
//   strcat((char*)currOutput, output);
//   currIndex += sizeof(output);
// }

void outputData() {
  // For debugging
  // for(int i=0;i< currIndex;i++) {
  //   Serial.print(currOutput[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println();
  // for(int i=0;i< currIndex;i++) {
  //   if (currOutput[i] > 0x1F && currOutput[i] < 0x7F) {
  //     char myChar = currOutput[i];
  //     Serial.print(myChar);
  //   } else {
  //     Serial.print("?");
  //   }
  // }
  // Serial.println();
  digitalWrite(15, LOW);
  if (lora.Send(currOutput, currIndex, SX126x_TXMODE_ASYNC)) {
    //addThenOutput("Send success");
  } else {
    //myFile.println("Send fail");
  }
  digitalWrite(15, HIGH);
  digitalWrite(SDCS, LOW);
  myFile = SD.open("/star.txt", FILE_APPEND);
  //myFile.println((char *)currOutput);

  for (int i = 0; i < index; i++) {
    myFile.print((char)currOutput[i]);
  }
  myFile.println();
  
  myFile.close();
  digitalWrite(SDCS, HIGH);
  memset(currOutput, 0, 255);
  currIndex = 0;
}
