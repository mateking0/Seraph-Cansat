#include <SD.h>
#include <SPI.h>
#include <Wire.h>             // include Wire library, required for I2C devices
#include <Adafruit_Sensor.h>  // include Adafruit sensor library
#include <Adafruit_BMP280.h>  // include adafruit library for BMP280 sensor
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SCD30.h>
#include <RH_RF95.h>
#include <Adafruit_GPS.h>

#define GPSSerial Serial1




Adafruit_SCD30  scd30;
unsigned long myTime;

File myFile;
Adafruit_GPS GPS(&GPSSerial);

 
// define device I2C address: 0x76 or 0x77 (0x77 is library default address)
#define BMP280_I2C_ADDRESS  0x76

#define RFM95_CS 9
#define RFM95_INT 10
#define RFM95_RST 6

#define RF95_FREQ 868.75
RH_RF95 rf95(RFM95_CS, RFM95_INT);


Adafruit_BMP280 bmp280;
Adafruit_MPU6050 mpu;

bool bmpaktiv = 0;
bool mpuaktiv = 0;
bool sdaktiv = 0;
bool CO2aktiv = 0;

float temperature = 0;  // get temperature
  float pressure    = 0;    // get pressure
  float altitude_   = 0; // get altitude (this should be adjusted to your local forecast)
String data = "";
char c;
unsigned long timee = 0;

float scdtemp = 0;
float scdhumidity = 0;
float scdco2 = 0;

float lat = 0;
float lon = 0;
float hig = 0;

void bmpfelall(){
  Serial.begin(9600);
  delay(1500);
  Serial.println("Bmp280 teszt");
  if (!bmp280.begin(BMP280_I2C_ADDRESS))
  {  
    Serial.println("Nincs csatlakoztatva bmp280 szenzor!");
    return;
  }
  Serial.println("Bmp280 csatlakoztatva.");
  bmpaktiv = 1;
  
}

void mpufelall(){
  Serial.begin(115200);
  Serial.println("Adafruit MPU6050 teszt!");
  delay(500);
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Nincs MPU6050 csatlakoztatva");
    return;
  }
  Serial.println("MPU6050 csatlakoztatva");
  mpuaktiv = 1;
}

void sdkartyaindit(){
  Serial.println("SD kártya indul");

  if (!SD.begin(4)) {
    Serial.println("Nem sikerült elindítani az sd kártyát!");
    return;
  }
  Serial.println("Sikeres SD kártya indítás!");
  

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("teszt6.txt", FILE_WRITE);
  if(myFile) {
    sdaktiv = 1;
    return;
  }
  Serial.println("Nem sikerült elindítani az sd kártyát!");
}

void co2indit() {
  Serial.println("Adafruit SCD30 test!");
  
  delay(1500);
  // Try to initialize!
  if (!scd30.begin()) {
    Serial.println("Failed to find SCD30 chip");
    return;
    
  }
  Serial.println("SCD30 Found!");
  CO2aktiv = true;


  // if (!scd30.setMeasurementInterval(10)){
  //   Serial.println("Failed to set measurement interval");
  //   while(1){ delay(10);}
  // }
  Serial.print("Measurement Interval: "); 
  Serial.print(scd30.getMeasurementInterval()); 
  Serial.println(" seconds");
}

void setup() {
  Serial.begin(9600);
  delay(1500);
  bmpfelall();
  mpufelall();
  sdkartyaindit();
  co2indit();
  //Serial.begin(9600);

 if (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  //Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("Set frequency failed");
    while (1);
  }
  Serial.print("Frequency: ");
  Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);


Serial.println("Adafruit GPS library basic test!");
//Serial.begin(9600);
  // Turn on GPS module
  GPSSerial.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  GPSSerial.println(PMTK_Q_RELEASE);
}





void bmpfuttat(){
  myTime = millis();
  temperature = bmp280.readTemperature();  // get temperature
  pressure    = bmp280.readPressure();     // get pressure
  altitude_   = bmp280.readAltitude(1012); // get altitude (this should be adjusted to your local forecast)
 

  // print data on the serial monitor 
  // 1: print temperature
 //myFile.print("Idő = ");
  myFile.print(myTime);
  //myFile.print(" milis");
  myFile.print(",");

  // 2: print temperature
  //myFile.print("Hőmérséklet = ");
  myFile.print(temperature);

  //myFile.println(" °C");
  myFile.print(",");

  // 3: print pressure
 // myFile.print("Légnyomás    = ");
  myFile.print(pressure);

  //myFile.println(" Pa");
  myFile.print(",");

  // 4: print altitude
 //myFile.print("Becsült magasság = ");
  myFile.print(altitude_);

  //myFile.println(" m");
  myFile.print(",");
  
 // myFile.println();  // start a new line
  //Serial.print("Idő = ");
  Serial.print(myTime);
  //Serial.print(" milis");
  Serial.print(",");
  // 2: print temperature
  //Serial.print("Hőmérséklet = ");
  Serial.print(temperature);
  Serial.print(",");
  //Serial.println(" °C");
  // 3: print pressure
  //Serial.print("Légnyomás    = ");
  Serial.print(pressure);
  Serial.print(",");
  //Serial.println(" Pa");
  // 4: print altitude
  //Serial.print("Becsült magasság = ");
  Serial.print(altitude_);
  Serial.print(",");
  //Serial.println(" m");
  
  //Serial.println();
}
sensors_event_t a, g, temp;
void mpufuttat() {
  /* Get new sensor events with the readings */
 // sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  //myFile.print("Gyorsulás X: ");
  myFile.print((a.acceleration.x)/9.81);
  myFile.print(",");
  //myFile.print(" G");
  //myFile.print(", Y: ");
  myFile.print((a.acceleration.y)/9.81);
  myFile.print(",");
  //myFile.print(" G");
  //myFile.print(", Z: ");
  myFile.print((a.acceleration.z)/9.81);
  myFile.print(",");
  //myFile.println(" G");
  //myFile.print("Teljes gyorsulás: ");
  myFile.print(abs((a.acceleration.x)/9.81)+abs((a.acceleration.y)/9.81)+abs((a.acceleration.z)/9.81));
 // myFile.println(" G");
  myFile.print(",");
  //myFile.print("Hőmérséklet: ");
  myFile.print(temp.temperature);
  //myFile.println(" fok");
 // myFile.print(",");

  myFile.println("");
  
  //Serial.print("Gyorsulás X: ");
  Serial.print((a.acceleration.x)/9.81);
  Serial.print(",");
  //Serial.print(" G");
  //Serial.print(", Y: ");
  Serial.print((a.acceleration.y)/9.81);
  Serial.print(",");
  //Serial.print(" G");
  //Serial.print(", Z: ");
  Serial.print((a.acceleration.z)/9.81);
  Serial.print(",");
  //Serial.println(" G");
  //Serial.print("Teljes gyorsulás: ");
  Serial.print(abs((a.acceleration.x)/9.81)+abs((a.acceleration.y)/9.81)+abs((a.acceleration.z)/9.81));
  Serial.print(",");
  //Serial.println(" G");
  //Serial.print("Hőmérséklet: ");
  Serial.print(temp.temperature);
  Serial.print(",");
  //Serial.println(" fok");

  Serial.println("");
}




void sdbezar(){
  myFile.close();
}






void CO2futtat(){
  if (scd30.dataReady()){
   // Serial.println("Data available!");

    if (!scd30.read()){ Serial.println("Error reading sensor data"); return; }

    scdtemp = scd30.temperature;
    scdhumidity = scd30.relative_humidity;
    scdco2 = scd30.CO2;

    Serial.print(scdtemp);
    Serial.print(",");
    Serial.print(scdhumidity);
    Serial.print(",");
    Serial.print(scdco2, 3);
    Serial.println();
    
    myFile.print(millis());
    myFile.print(",");
    myFile.print(scdtemp);
    myFile.print(",");
    myFile.print(scdhumidity);
    myFile.print(",");
    myFile.print(scdco2, 3);
    myFile.println();
  } else {
    //Serial.println("No data");
  }
}

void loop() {
  
  
  for (int i = 0; i < 800; i++){
    c = GPS.read();
    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA())) // Continue parsing if data is not yet complete
        return;

      // Check if data is valid
      if (!GPS.fix) {
        Serial.println("Waiting for valid data...");
        return;
      }
      lat = GPS.latitudeDegrees;
      lon = GPS.longitudeDegrees;
      hig = GPS.altitude;
      // Print location data
      Serial.print("Location: ");
      Serial.print(lat, 6);
      Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(lon, 6);
      Serial.println(GPS.lon);
      Serial.println(hig, 2);
      break;
    }
  }

  myTime = millis();
  timee = myTime/1002;
  Serial.print(timee);
  Serial.print(",");
  myFile = SD.open("bmp_mpu.txt", FILE_WRITE);

  if(bmpaktiv){
    bmpfuttat();
   
  }
  if(mpuaktiv){
    mpufuttat();
   
  }
  myFile.close();
  myFile = SD.open("scd30.txt", FILE_WRITE);
  
  
  if(CO2aktiv){
    CO2futtat();
  }
  myFile.close();
  

  //delay(500);
  myFile = SD.open("gps.txt", FILE_WRITE);
  myFile.print(lat,6);
  myFile.print(",");
  myFile.print(hig, 2);
  myFile.print(",");
  myFile.println(lon,6);
  myFile.close();


  myFile = SD.open("data.txt", FILE_WRITE);

  data = String(millis()) + "," + String(pressure) + "," + String(altitude_) + "," + String(temperature) + "," 
                + String(a.acceleration.x/9.81) + "," + String(a.acceleration.y/9.81) + "," + String(a.acceleration.z/9.81)
                + "," + String(temp.temperature) + ","  + String(scdco2, 3) + 
                "," + String(scdhumidity) + "," + String(scdtemp, 2) + "," + String(lat, 6) + "," + String(hig, 2) + "," + String(lon, 6);
  
  Serial.println(data);
  myFile.println(data);
  myFile.close();

  if (rf95.send((uint8_t *)data.c_str(), data.length())) {
    Serial.println("Sent data");
  } else {
    Serial.println("Failed to send data");
  }
  data = "";
  delay(900);

}
