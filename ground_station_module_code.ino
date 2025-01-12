#include <SPI.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <SD.h>

File myFile;

// For Feather M0 Adalogger
#define LED 8
#define RFM95_CS 9
#define RFM95_INT 10
#define RFM95_RST 6

#define RF95_FREQ 868.75

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

String data = "";
String rs = "";

void setup() 
{
  pinMode(LED, OUTPUT);     
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  while (!Serial); // wait until serial console is open, remove if not tethered to computer
  Serial.begin(9600);
  delay(100);
  Serial.println("Feather LoRa TX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

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
  
  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
  }
  Serial.println("initialization done.");
}

void loop() 
{
  // Send a message to RX
  /*Serial.print("Futasido ");
  Serial.println(millis());

  uint8_t data[5] = { 0x00, 0x01, 0x02, 0x03, 0x04 };
  rf95.send(data, sizeof(data));

  digitalWrite(LED, HIGH);
  delay(10);
  digitalWrite(LED, LOW);
  delay(1000);*/

  // Wait for a reply
  myFile = SD.open("data.txt", FILE_WRITE);

  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf95.waitAvailableTimeout(1000)) {
    // Should be a reply message for us now
    if (rf95.recv(buf, &len)) {
      //Serial.print("Got reply: ");
      rs = rf95.lastRssi();
      data = rs + "," + (char*)buf;
      Serial.println(data);
      myFile.println(data);
      //Serial.print("RSSI: ");
      //Serial.println(rf95.lastRssi(), DEC);
    }
    else {
      Serial.println("Receive failed");
    }
  }
  myFile.close();
}