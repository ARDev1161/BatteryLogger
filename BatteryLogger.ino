// CONNECTIONS:
// DS1302 CLK/SCLK --> 3
// DS1302 DAT/IO --> 4
// DS1302 RST/CE --> 2
// Vbat sensor --> A6

#include <SPI.h>
#include <SD.h>
#include <ThreeWire.h>
#include <RtcDS1302.h>
#include "MPU6050.h"

//SD section
#define chipSelect 10

//DS1302 section
#define countof(a) (sizeof(a) / sizeof(a[0]))
ThreeWire myWire(4,3,2); // IO, SCLK, CE
RtcDS1302<ThreeWire> Rtc(myWire);
RtcDateTime now ;

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//MPU6050 section
MPU6050 accelgyro; //MPU6050 accelgyro(0x69); // <-- use for AD0 high
int16_t ax, ay, az;
int16_t gx, gy, gz;
#define OUTPUT_READABLE_ACCELGYRO //tab-separated list values in decimal.
//#define OUTPUT_BINARY_ACCELGYRO //all 6 axes of data as 16-bit binary, one right after the other.

//Voltage sensor section
#define batVin 6
#define R1 99900.0 // resistance of R1 (100K) - 99900
#define R2 31650.0 // resistance of R2 (32K) - 31650

void mpu6050Init(){
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  // initialize device
  accelgyro.initialize();
  delay(1000);

  // verify connection
  Serial.print("mpu6050 test:\t");
  Serial.println(accelgyro.testConnection());

  // use the code below to change accel/gyro offset values
  /*
  Serial.println("Updating internal sensor offsets...");
  // -76  -2359 1688  0 0 0
  Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
  Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
  Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
  Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
  Serial.print("\n");
  accelgyro.setXGyroOffset(220);
  accelgyro.setYGyroOffset(76);
  accelgyro.setZGyroOffset(-85);
  Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
  Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
  Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
  Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
  Serial.print("\n");
  */    
}

void mpuProc(){
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        Serial.print("a/g:\t");
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.println(gz);
    #endif

    #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    #endif
}

void timeSetup(){
    Serial.print("\ncompiled: ");
    Serial.print(__DATE__);
    Serial.println(__TIME__);
    
    Rtc.Begin();

    RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
    //printDateTime(compiled);

    if (!Rtc.IsDateTimeValid()){
        //Serial.println("RTC lost confidence in the DateTime!");
        Rtc.SetDateTime(compiled);
    }
    if (Rtc.GetIsWriteProtected()){
        //Serial.println("RTC was write protected, enabling writing now");
        Rtc.SetIsWriteProtected(false);
    }
    if (!Rtc.GetIsRunning()){
        //Serial.println("RTC was not actively running, starting now");
        Rtc.SetIsRunning(true);
    }
    
    now = Rtc.GetDateTime();
    if (now < compiled){
        Serial.println("RTC is older than compile time!  (Updating DateTime)");
        Rtc.SetDateTime(compiled);
    }
    else if (now > compiled){
        //Serial.println("RTC is newer than compile time. (this is expected)");
    }
    else if (now == compiled){
        //Serial.println("RTC is the same as compile time! (not expected but all is fine)");
    }
}

void printDateTime(const RtcDateTime& dt){
    char datestring[20];
    snprintf_P(datestring, 
            countof(datestring),
            PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
            dt.Month(),
            dt.Day(),
            dt.Year(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );
    Serial.println(datestring);
}

String getDateStamp(const RtcDateTime& dt){
    String dateStamp;
    
    dateStamp+=dt.Year();
    
    if(dt.Month()<10){
      dateStamp+="0";
      dateStamp+=dt.Month();
    }
    else{
      dateStamp+=dt.Month();
      }
    
    if(dt.Day()<10){
      dateStamp+="0";
      dateStamp+=dt.Day();
    }
    else{
      dateStamp+=dt.Day();
      }
    
    return dateStamp;
}

String confirmLogName(){
  String filename="";
  now = Rtc.GetDateTime();
  filename = getDateStamp(now);

  if (SD.exists(filename))
    for(int i=1;i<=10000;i++){
      if(!SD.exists(filename +"."+ i)){
        filename+=".";
        filename+=i;
        break;
      }
    }    
    
  return filename;
}

bool checkDateOfFile(String fileName){
  bool checked = true;
  
  if(getDateStamp(now).compareTo(fileName.substring(0,8)) != 0)
    checked = false;
    
  return checked;
}

String timeProc(){
  String timeStamp = "";
    now = Rtc.GetDateTime();  

    timeStamp=now.Hour();
    timeStamp+=":";
    timeStamp+=now.Minute();
    timeStamp+=":";
    timeStamp+=now.Second();
    
    return timeStamp;
}

float getVoltage(){
  
  // read the value at analog input
  float vbat = (analogRead(batVin) * 5.0) / 1024.0; // see text
  vbat /= R2/(R1+R2);
  
  if (vbat<0.09)
    vbat=0.0;//statement to quash undesired reading !
  return vbat;
}

void logger(String logName){
  String dataString = "";

  if(!SD.exists(logName)){
    while(checkDateOfFile(logName)){
      File logFile = SD.open(logName, FILE_WRITE);
    
      // if the file is available, write to it:
      if (logFile) {        
          dataString=timeProc();                      //write timeStamp        
          dataString+="\t";
          dataString+=getVoltage();                   //write voltage
          dataString+="\t";
          dataString+=(accelgyro.getTemperature())/340.00+36.53;     //write temperature
          logFile.println(dataString);
          logFile.close();
        
         // Serial.print("wr\t");
         // Serial.println(dataString);

          delay(20);
      }
      else {
        Serial.println("error opening file");
      }
    }
  }
}

void setup() {
  Serial.begin(9600);                                         // инициализируем работу с аппаратным последовательным портом и указываем скорость работы 9600 бод
  
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  
  timeSetup();
  mpu6050Init();
}

void loop() {
  String logName="";
  logName = confirmLogName();
  Serial.println("\nfilename available\t"+logName);
  logger(logName);
}
