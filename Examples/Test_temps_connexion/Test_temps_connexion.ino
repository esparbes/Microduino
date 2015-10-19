#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include "utility/debug.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

// These are the interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   2  // MUST be an interrupt pin!
// These can be any two pins
#define ADAFRUIT_CC3000_VBAT  9
#define ADAFRUIT_CC3000_CS    10
// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                         SPI_CLOCK_DIV2); // you can change this clock speed

#define ECOLE

#ifdef ECOLE
#define WLAN_SSID       "Robogen_ESPARBES"           // cannot be longer than 32 characters!
#define WLAN_PASS       "Ihtqgrd797tf4UTa"
// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_WPA2
#endif

#ifdef MAISON
#define WLAN_SSID       "CitycableMax"           // cannot be longer than 32 characters!
#define WLAN_PASS       "97tf4UTaIhtqgrd7"
// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_WPA2
#endif

#define LISTEN_PORT           23    // What TCP port to listen on for connections.  The echo protocol uses port 7.

Adafruit_CC3000_Server server(LISTEN_PORT);
MPU6050 accelgyro;

int pinLed = 6, pinTemp = A5;
float temp;

uint8_t ch;
int16_t ax, ay, az;
int16_t gx, gy, gz;

//********************************************************************************************

void setup(void)
{
  pinMode(pinLed, OUTPUT);
  pinMode(pinTemp, INPUT);
  digitalWrite(pinLed, HIGH);
  delay(500);
  digitalWrite(pinLed, LOW);

  Serial.begin(115200);

// ***************** Configuration de la connexion Wifi ******************************
  
  displayDriverMode();
  Serial.print("Free RAM: "); Serial.println(getFreeRam(), DEC);

  /* Initialise the module */
  Serial.println(F("Initializing..."));
  if (!cc3000.begin())
  {
    Serial.println(F("Couldn't begin()! Check your wiring?"));
    while (1);
  }
  
  uint16_t firmware = checkFirmwareVersion();
  if ((firmware != 0x113) && (firmware != 0x118)) {
    Serial.println(F("Wrong firmware version!"));
    for(;;);
  }
  
  displayMACAddress();

  #ifndef CC3000_TINY_DRIVER
  listSSIDResults();
#endif

    /* Delete any old connection data on the module */
  Serial.println(F("\nDeleting old connection profiles"));
  if (!cc3000.deleteProfiles()) {
    Serial.println(F("Failed!"));
    while(1);
  }

  /* Attempt to connect to an access point */
  char *ssid = WLAN_SSID;             /* Max 32 chars */
  Serial.print(F("\nAttempting to connect to ")); Serial.println(ssid);

  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("Failed!"));
    while (1);
  }

  Serial.println(F("Connected!"));

  Serial.println(F("Request DHCP"));
  while (!cc3000.checkDHCP())
  {
    delay(100); // ToDo: Insert a DHCP timeout!
  }

  /* Display the IP address DNS, Gateway, etc. */
  while (! displayConnectionDetails()) {
    delay(1000);
  }

  //*************************** Configuration de l'IMU *********************************

   Wire.begin();
   Serial.println("Initializing I2C devices...");
   accelgyro.initialize();
   Serial.println("Testing device connections...");
   Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  //********************** Ecoute des eventuels clients ********************************

  // Start listening for connections
  server.begin();

  Serial.println(F("Listening for connections..."));
  int16_t tpsLoop = millis();
  Serial.print("Le loop s'ouvre a :");
  Serial.print(tpsLoop);
  Serial.println(" ms");
}



void loop() {
  temp = temperatura();
  int16_t temps_2;
  int16_t temps_3;
  int16_t temps_4;
  int16_t tempsExeComMic;
  int16_t tempsExeMicCom;
  int16_t tempsExeTotal;

  int16_t tempsCumMicCom = 0;
  int16_t tempsMoyMicCom;

  
  int16_t temps_1 = millis();
      
  // Try to get a client which is connected.
  Adafruit_CC3000_ClientRef client = server.available();
  if (client) {
    // Check if there is data available to read.
    if (client.available() > 0) {
      // Read a byte and write it to all clients.
      ch = client.read();

      switch (ch) {
        case '0':

          temps_2 = millis();
          
          client.println("Accelerometre et Gyroscope"); //Plus ce qu'on doit envoyer est long, plus le délai est elevé
          
          for(int k=0; k<30; k++){            
              
              temps_3 = millis();   //Temps AVANT l'envoi de données 
              accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
                                 
              client.print("ax : "); client.println(ax); 
              client.print("ay : "); client.println(ay);
              client.print("az : "); client.println(az);
              client.print("gx : "); client.println(gx);
              client.print("gy : "); client.println(gy);
              client.print("gz : "); client.println(gz);
              client.print("Iteration : ");
              client.println(k);
              
              temps_4 = millis();  // Temps APRES l'envoi de données
              
              tempsExeComMic = temps_2 - temps_1;
              tempsExeMicCom = temps_4 - temps_3;    //Temps de chaque transmission
              tempsCumMicCom = tempsCumMicCom + tempsExeMicCom; //Temps cumulé de ces transmissions
              //tempsExeTotal = temps_4 - temps_1;
              client.print("temps Comput-Micro :");
              client.println(tempsExeComMic);
              client.print("temps Micro-Comput :");
              client.println(tempsExeMicCom);
              //client.print("temps execution total :");
              //client.println(tempsExeTotal);
              client.println("");
          }
          tempsMoyMicCom = tempsCumMicCom/30; //Temps moyen de transmission Microduino->Computer
          client.print("Temps Micro-Comput moyen : ");
          client.println(tempsMoyMicCom);   
          break;
      }
    }
  }
}



//*******************************************************************************************

bool displayConnectionDetails(void)
{
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;

  if (!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
  {
    Serial.println(F("Unable to retrieve the IP Address!rn"));
    return false;
  }
  else
  {
    Serial.print(F("nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
    Serial.print(F("nNetmask: ")); cc3000.printIPdotsRev(netmask);
    Serial.print(F("nGateway: ")); cc3000.printIPdotsRev(gateway);
    Serial.print(F("nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
    Serial.print(F("nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    Serial.println();
    return true;
  }
}

float temperatura() {

  float Vout;
  float res = 0.010; // en V/ºC

  Vout = analogRead(pinTemp) * 0.00488758553275; // step * V/step = V
  temp = (Vout / res ) - 2.5; // V/(V/ºC) = ºC
  return temp;

}

//************************************************************************************************

void displayDriverMode(void)
{
  #ifdef CC3000_TINY_DRIVER
    Serial.println(F("CC3000 is configure in 'Tiny' mode"));
  #else
    Serial.print(F("RX Buffer : "));
    Serial.print(CC3000_RX_BUFFER_SIZE);
    Serial.println(F(" bytes"));
    Serial.print(F("TX Buffer : "));
    Serial.print(CC3000_TX_BUFFER_SIZE);
    Serial.println(F(" bytes"));
  #endif
}

//**************************************************************************************************

void displayMACAddress(void)
{
  uint8_t macAddress[6];
  
  if(!cc3000.getMacAddress(macAddress))
  {
    Serial.println(F("Unable to retrieve MAC Address!\r\n"));
  }
  else
  {
    Serial.print(F("MAC Address : "));
    cc3000.printHex((byte*)&macAddress, 6);
  }
}

//***************************************************************************************************

uint16_t checkFirmwareVersion(void)
{
  uint8_t major, minor;
  uint16_t version;
  
#ifndef CC3000_TINY_DRIVER  
  if(!cc3000.getFirmwareVersion(&major, &minor))
  {
    Serial.println(F("Unable to retrieve the firmware version!\r\n"));
    version = 0;
  }
  else
  {
    Serial.print(F("Firmware V. : "));
    Serial.print(major); Serial.print(F(".")); Serial.println(minor);
    version = major; version <<= 8; version |= minor;
  }
#endif
  return version;
}

//****************************************************************************************************

void listSSIDResults(void)
{
  uint8_t valid, rssi, sec, index;
  char ssidname[33]; 

  index = cc3000.startSSIDscan();

  Serial.print(F("Networks found: ")); Serial.println(index);
  Serial.println(F("================================================"));

  while (index) {
    index--;

    valid = cc3000.getNextSSID(&rssi, &sec, ssidname);
    
    Serial.print(F("SSID Name    : ")); Serial.print(ssidname);
    Serial.println();
    Serial.print(F("RSSI         : "));
    Serial.println(rssi);
    Serial.print(F("Security Mode: "));
    Serial.println(sec);
    Serial.println();
  }
  Serial.println(F("================================================"));

  cc3000.stopSSIDscan();
}
